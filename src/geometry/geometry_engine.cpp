#include "dcs_nav_plugin/geometry/geometry_engine.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include "polypartition.h" 
#include "rclcpp/rclcpp.hpp"

// Tag for logging
static const char* TAG = "geometry_engine";

namespace dcs_nav_plugin
{

GeometryEngine::GeometryEngine()
: poly_epsilon_(0.05),
  min_obstacle_area_(0.01),
  inflation_radius_(0.0),
  top_k_(5),
  dilation_pixels_(1)
{
}

GeometryEngine::~GeometryEngine()
{
}

void GeometryEngine::configure(
  double poly_epsilon, 
  double obstacle_height_thres,
  double inflation_radius,
  int top_k,
  int dilation_pixels)
{
  poly_epsilon_ = poly_epsilon;
  // min_obstacle_area_ ... 
  inflation_radius_ = inflation_radius;
  top_k_ = top_k;
  dilation_pixels_ = dilation_pixels;
  
  RCLCPP_INFO(rclcpp::get_logger(TAG), "========== GeometryEngine Configured ==========");
  RCLCPP_INFO(rclcpp::get_logger(TAG), "poly_epsilon: %.3f", poly_epsilon);
  RCLCPP_INFO(rclcpp::get_logger(TAG), "inflation_radius: %.3f", inflation_radius);
  RCLCPP_INFO(rclcpp::get_logger(TAG), "top_k: %d", top_k);
  RCLCPP_INFO(rclcpp::get_logger(TAG), "dilation_pixels: %d", dilation_pixels);
  RCLCPP_INFO(rclcpp::get_logger(TAG), "==============================================");
}

std::vector<Polygon> GeometryEngine::extractObstacles(
  nav2_costmap_2d::Costmap2D * costmap,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> & ref_path,
  std::vector<Polygon> & all_polygons_out)
{
  all_polygons_out.clear();
  
  // FORCE LOGGING
  // FORCE LOGGING
  // RCLCPP_INFO_THROTTLE(rclcpp::get_logger(TAG), *(costmap->getClock()), 1000, "extractObstacles Called. Costmap Ptr: %p", costmap);
  // Actually can't easily get clock from costmap ptr here without dep, just use INFO every time (it's called every loop though)
  RCLCPP_INFO(rclcpp::get_logger(TAG), "[GeometryEngine] extractObstacles Called");

  if (!costmap) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] ERROR: Costmap is NULL!");
    return {};
  }

  try {
    // 1. 获取所有轮廓
    std::vector<std::vector<cv::Point>> contours = getContoursFromCostmap(costmap);
    
    RCLCPP_INFO(rclcpp::get_logger(TAG), "[GeometryEngine] Contours found: %zu", contours.size());
    
    // 2. 凸分解
    double resolution = costmap->getResolution();
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();

    // 预分配内存
    std::vector<Obstacle> candidate_obstacles;
    candidate_obstacles.reserve(contours.size() * 2);
    all_polygons_out.reserve(contours.size() * 2);

    for (const auto & contour : contours) {
      // 简单的面积过滤 (Pixel coords)
      double area = cv::contourArea(contour);
      // DEBUG LOG SMALL AREAS
      // if (area < 5.0) {
      //   std::cerr << "[GeometryEngine] 跳过小轮廓: 面积=" << area << "\n" << std::flush;
      //   continue;
      // }
      // [Fix] Removed area filter completely.
      // Even 0-area contours (lines/points) are valid obstacles in navigation.
      // if (area < 0.5) continue; 

      // 进行凸分解
      auto convex_polys = decomposeConvex(contour, resolution, origin_x, origin_y);
      
      // 对每个凸多边形，计算质心和到路径的距离
      for (const auto & poly : convex_polys) {
        if (poly.empty()) continue;
        
        Obstacle obs;
        obs.polygon = poly;
        
        // 计算质心
        double sum_x = 0, sum_y = 0;
        for (const auto & pt : poly) {
          sum_x += pt.x;
          sum_y += pt.y;
        }
        obs.centroid.x = sum_x / poly.size();
        obs.centroid.y = sum_y / poly.size();
        obs.centroid.z = 0.0;
        
        // 计算到参考路径的距离
        double min_vertex_dist = std::numeric_limits<double>::max();
        for (const auto& pt : poly) {
            double d = computeDistanceToPath(pt, ref_path);
            if (d < min_vertex_dist) min_vertex_dist = d;
        }
        obs.dist_to_path = min_vertex_dist;
        
        // DEBUG: print every obstacle detail
        // std::cerr << "[GeometryEngine] 障碍物: 质心=(" << obs.centroid.x << "," << obs.centroid.y 
        //           << "), 到路径=" << obs.dist_to_path << "m, 顶点=" << poly.size() << "\n" << std::flush;
        
        candidate_obstacles.push_back(obs);
        all_polygons_out.push_back(poly);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger(TAG), "[GeometryEngine] 总障碍物: %zu, Top-K=%d", candidate_obstacles.size(), top_k_);


    // 3. Top-K 排序筛选
    try {
        if (candidate_obstacles.size() > (size_t)top_k_ && top_k_ > 0) {
          std::partial_sort(
            candidate_obstacles.begin(),
            candidate_obstacles.begin() + top_k_,
            candidate_obstacles.end(),
            [](const Obstacle & a, const Obstacle & b) {
              return a.dist_to_path < b.dist_to_path;
            });
          
          candidate_obstacles.resize(top_k_);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] Sorting Exception: %s. Using unsorted list.", e.what());
        // Do not return empty, just proceed with unsorted list (better than nothing)
    }

    // 4. 返回结果 - Simplified logic to avoid nested try-catch issues
    std::vector<Polygon> result;
    result.reserve(candidate_obstacles.size());
    for (const auto & obs : candidate_obstacles) {
      result.push_back(obs.polygon);
    }
    
    return result;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] extractObstacles 异常: %s", e.what());
    // Return all_polygons_out as emergency fallback (already populated)
    std::vector<Polygon> fallback_result;
    fallback_result.reserve(all_polygons_out.size());
    for (const auto& poly : all_polygons_out) {
        fallback_result.push_back(poly);
    }
    return fallback_result;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] extractObstacles 未知异常");
    // Return all_polygons_out as emergency fallback
    std::vector<Polygon> fallback_result;
    fallback_result.reserve(all_polygons_out.size());
    for (const auto& poly : all_polygons_out) {
        fallback_result.push_back(poly);
    }
    return fallback_result;
  }
}

std::vector<std::vector<cv::Point>> GeometryEngine::getContoursFromCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  if (!costmap) return {};

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  
  if (size_x == 0 || size_y == 0 || size_x > 10000 || size_y > 10000) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] Invalid Costmap Size: %dx%d", size_x, size_y);
    return {};
  }
  
  unsigned char * char_map = costmap->getCharMap();
  if (!char_map) {
      RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] char_map is NULL");
      return {};
  }

  try {
    cv::Mat map_img(size_y, size_x, CV_8UC1);
    std::memcpy(map_img.data, char_map, size_x * size_y);
    
    // Stats
    // [Debug] Count pixel values to diagnose map data
    int count_253 = 0;
    int count_254 = 0;
    int count_255 = 0;
    for(size_t i=0; i<size_x*size_y; ++i) {
        if (char_map[i] == 253) count_253++;
        else if (char_map[i] == 254) count_254++;
        else if (char_map[i] == 255) count_255++;
    }
    RCLCPP_INFO(rclcpp::get_logger(TAG), "[GeometryEngine] Map Pixel Stats: 253(Inscribed)=%d, 254(Lethal)=%d, 255(Unknown)=%d", 
        count_253, count_254, count_255);

    // [Fix] User Request: Only extract Lethal Obstacles (254) and Unknown (255)
    // Ignore 253 (Inscribed) and lower costs (Inflation) by setting threshold to 253
    unsigned char thresh_value = 253;

    cv::Mat bin_img;
    // 使用高阈值提取核心障碍物
    // Costmap: > 253 => 254(Lethal), 255(Unknown)
    cv::threshold(map_img, bin_img, thresh_value, 255, cv::THRESH_BINARY);

    // [Fix] User Request: Expand by configurable pixels (Dilation)
    // If dilation_pixels_ > 0, we dilate. k_size = 2*r + 1
    if (dilation_pixels_ > 0) {
        int k_size = 2 * dilation_pixels_ + 1;
        cv::dilate(bin_img, bin_img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_size, k_size)));
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    return contours;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] getContoursFromCostmap 异常: %s", e.what());
    return {};
  } catch (...) {
    return {};
  }
}


std::vector<Polygon> GeometryEngine::decomposeConvex(
  const std::vector<cv::Point> & contour, 
  double resolution, 
  double origin_x, 
  double origin_y)
{
  if (contour.empty() || contour.size() < 3) return {};
  
  size_t contour_size = contour.size();
  (void)contour_size; // Supress unused warning

  try {
    // 1. 简化轮廓
    std::vector<cv::Point> approx_contour;
    double epsilon_pixels = poly_epsilon_ / resolution;
    // [Fix] Remove forced minimum epsilon (was 2.0). 
    // For small obstacles (table legs < 10cm), 2.0 pixels (10cm) error removes them.
    // Use calculated value (e.g. 1.0 pixels) to preserve shape.
    // User requested 1.0 pixel (approx 5cm) with fallback for robustness.
    if (epsilon_pixels < 1.0) epsilon_pixels = 1.0; // Min 1.0 pixel to balance perf/accuracy
    
    cv::approxPolyDP(contour, approx_contour, epsilon_pixels, true);
    
    // [Fix] Fallback: If approximation is degenerate (<3 points), use convex hull of original contour
    // This ensures even a 3-pixel triangle is preserved instead of being discarded
    if (approx_contour.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger(TAG), "[GeometryEngine] Approx too small (%zu), using ConvexHull of raw contour.", approx_contour.size());
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        approx_contour = hull;
        if (approx_contour.size() < 3) return {}; // Still degenerate? Give up.
    }
    
    // SAFETY: If too many points, force convex hull directly to avoid TPP crash or hang
    bool force_hull = false;
    // Lower threshold to 40 to prevent bad_alloc on complex shapes
    // Reduced to 20 to prefer simpler hulls over complex decompositions for navigation
    // [Fix] Increased to 2000 to allow complex room contours to undergo decomposition instead of hulling
    // [Fix-Crash] 2000 is too high and allows degenerate shapes to crash TPPL (bad_array_new_length).
    // Reduced to 300: Enough for rooms, filters noise.
    if (approx_contour.size() > 300) {
        RCLCPP_WARN(rclcpp::get_logger(TAG), "[GeometryEngine] Contour points > 300 (%zu), forcing Convex Hull (Safety).", approx_contour.size());
        force_hull = true;
    }

    // 2. Convex Decomposition
    std::vector<Polygon> result_polys;

    if (!force_hull) {
      try {
        // 2.1 Use TPPL to handle orientation robustness
        // Do NOT rely on OpenCV signed area due to coordinate system flip (Image Y-down vs World Y-up)
        
        TPPLPoly input_poly;
        input_poly.Init(static_cast<long>(approx_contour.size()));
        
        for (size_t i = 0; i < approx_contour.size(); ++i) {
          input_poly[static_cast<long>(i)].x = static_cast<tppl_float>(approx_contour[i].x);
          input_poly[static_cast<long>(i)].y = static_cast<tppl_float>(approx_contour[i].y);
        }
        input_poly.SetHole(false);
        
        // Force Correct Orientation
        // This method checks current orientation and inverts if necessary to match requested
        input_poly.SetOrientation(TPPL_ORIENTATION_CCW); 
        
        TPPLPartition pp;
        TPPLPolyList convex_parts;
        
        if (pp.ConvexPartition_HM(&input_poly, &convex_parts) != 0) {
             for (auto& tppl_poly : convex_parts) {
                long n_pts = tppl_poly.GetNumPoints();
                if (n_pts < 3) continue;
                Polygon poly;
                poly.reserve(static_cast<size_t>(n_pts));
                for (long i = 0; i < n_pts; ++i) {
                    TPPLPoint& pt = tppl_poly[i];
                    geometry_msgs::msg::Point gpt;
                    gpt.x = origin_x + (pt.x + 0.5) * resolution;
                    gpt.y = origin_y + (pt.y + 0.5) * resolution;
                    gpt.z = 0.0;
                    poly.push_back(gpt);
                }
                result_polys.push_back(std::move(poly));
             }
        } else {
            force_hull = true; // Fallback
        }
      } catch (const std::bad_array_new_length &e) {
         RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] TPPL Memory Exception: %s. Fallback to Hull.", e.what());
         force_hull = true;
      } catch (const std::exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] TPPL Exception: %s. Fallback to Hull.", e.what());
          force_hull = true;
      } catch (...) {
          RCLCPP_ERROR(rclcpp::get_logger(TAG), "[GeometryEngine] TPPL Unknown Exception. Fallback to Hull.");
          force_hull = true;
      }
    }

    if (force_hull || result_polys.empty()) {
      // Fallback to simple Convex Hull
      std::vector<cv::Point> hull;
      cv::convexHull(approx_contour, hull, false, true);
      
      if (hull.size() >= 3) {
          Polygon poly;
          poly.reserve(hull.size());
          for (const auto& pt : hull) {
            geometry_msgs::msg::Point gpt;
            gpt.x = origin_x + (static_cast<double>(pt.x) + 0.5) * resolution;
            gpt.y = origin_y + (static_cast<double>(pt.y) + 0.5) * resolution;
            gpt.z = 0.0;
            poly.push_back(gpt);
          }
          result_polys.push_back(poly);
      }
    }
    
    return result_polys;
    
  } catch (...) {
    return {};
  }
}

double GeometryEngine::computeDistanceToPath(
  const geometry_msgs::msg::Point & pt, 
  const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  if (path.empty()) return std::numeric_limits<double>::max();

  double min_dist_sq = std::numeric_limits<double>::max();
  
  for (const auto & pose : path) {
    double dx = pt.x - pose.pose.position.x;
    double dy = pt.y - pose.pose.position.y;
    double d2 = dx*dx + dy*dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
    }
  }
  return std::sqrt(min_dist_sq);
}

}  // namespace dcs_nav_plugin
