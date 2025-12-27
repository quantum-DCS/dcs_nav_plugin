#include "dcs_nav_plugin/geometry/geometry_engine.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "polypartition.h" // Depending on include path setup

namespace dcs_nav_plugin
{

GeometryEngine::GeometryEngine()
: poly_epsilon_(0.05),
  min_obstacle_area_(0.01),
  inflation_radius_(0.0),
  top_k_(5)
{
}

GeometryEngine::~GeometryEngine()
{
  // std::cerr does not need to be closed
}

void GeometryEngine::configure(
  double poly_epsilon, 
  double obstacle_height_thres,
  double inflation_radius,
  int top_k)
{
  poly_epsilon_ = poly_epsilon;
  // min_obstacle_area_ ... 
  inflation_radius_ = inflation_radius;
  top_k_ = top_k;
  
  // Output to stderr for visibility in main logs
  std::cerr << "\n========== GeometryEngine Configured ==========\n";
  std::cerr << "poly_epsilon: " << poly_epsilon << "\n";
  std::cerr << "inflation_radius: " << inflation_radius << "\n";
  std::cerr << "top_k: " << top_k << "\n";
  std::cerr << "==============================================\n" << std::flush;
}

std::vector<Polygon> GeometryEngine::extractObstacles(
  nav2_costmap_2d::Costmap2D * costmap,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> & ref_path,
  std::vector<Polygon> & all_polygons_out)
{
  all_polygons_out.clear();
  if (!costmap) {
    return {};
  }

  try {
    // 1. 获取所有轮廓
    auto contours = getContoursFromCostmap(costmap);
    
    // 2. 凸分解
    double resolution = costmap->getResolution();
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();

    // 预分配内存，避免动态扩容导致的问题
    std::vector<Obstacle> candidate_obstacles;
    candidate_obstacles.reserve(contours.size() * 2);  // 预分配足够空间
    all_polygons_out.reserve(contours.size() * 2);

    for (const auto & contour : contours) {
      // 简单的面积过滤 (Pixel coords)
      double area = cv::contourArea(contour);
      if (area < 5.0) {
        std::cerr << "[GeometryEngine] 跳过小轮廓: 面积=" << area << "\n" << std::flush;
        continue;
      }

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
        
        // 计算到参考路径的距离 (使用顶点的最小距离，而非质心距离，以优先考虑靠近路径的大障碍物)
        double min_vertex_dist = std::numeric_limits<double>::max();
        for (const auto& pt : poly) {
            double d = computeDistanceToPath(pt, ref_path);
            if (d < min_vertex_dist) min_vertex_dist = d;
        }
        // 如果多边形很小，质心距离也可以，但为安全起见取最小值
        obs.dist_to_path = min_vertex_dist;
        
        std::cerr << "[GeometryEngine] 障碍物: 质心=(" << obs.centroid.x << "," << obs.centroid.y 
                  << "), 到路径=" << obs.dist_to_path << "m, 顶点=" << poly.size() << "\n" << std::flush;
        
        candidate_obstacles.push_back(obs);
        all_polygons_out.push_back(poly);
      }
    }

    std::cerr << "[GeometryEngine] 总障碍物: " << candidate_obstacles.size() << ", Top-K=" << top_k_ << "\n" << std::flush;


    // 3. Top-K 排序筛选
    if (candidate_obstacles.size() > (size_t)top_k_) {
      std::partial_sort(
        candidate_obstacles.begin(),
        candidate_obstacles.begin() + top_k_,
        candidate_obstacles.end(),
        [](const Obstacle & a, const Obstacle & b) {
          return a.dist_to_path < b.dist_to_path;
        });
      
      std::cerr << "[GeometryEngine] Top-" << top_k_ << " 障碍物（按到路径距离排序）：" << "\n" << std::flush;
      for (int i = 0; i < top_k_; ++i) {
        std::cerr << "  " << i << ": 质心=(" << candidate_obstacles[i].centroid.x << "," 
                  << candidate_obstacles[i].centroid.y << "), 距离=" << candidate_obstacles[i].dist_to_path << "\n" << std::flush;
      }
      
      candidate_obstacles.resize(top_k_);
    }

    // 4. 返回结果
    std::vector<Polygon> result;
    result.reserve(candidate_obstacles.size());
    for (const auto & obs : candidate_obstacles) {
      result.push_back(obs.polygon);
    }
    
    return result;
    
  } catch (const std::exception& e) {
    std::cerr << "[GeometryEngine] extractObstacles 异常: " << e.what() << "\n" << std::flush;
    return {};
  } catch (...) {
    std::cerr << "[GeometryEngine] extractObstacles 未知异常" << "\n" << std::flush;
    return {};
  }
}

std::vector<std::vector<cv::Point>> GeometryEngine::getContoursFromCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  if (!costmap) return {};

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  
  // 防止异常大小
  if (size_x == 0 || size_y == 0 || size_x > 10000 || size_y > 10000) {
    return {};
  }
  
  unsigned char * char_map = costmap->getCharMap();
  if (!char_map) return {};

  try {
    // 创建 OpenCV Mat - 拷贝数据而非引用，避免并发问题
    cv::Mat map_img(size_y, size_x, CV_8UC1);
    std::memcpy(map_img.data, char_map, size_x * size_y);
    
    // 统计像素值分布 (调试用)
    int count_lethal = 0, count_inscribed = 0, count_free = 0;
    unsigned char max_val = 0;
    for (unsigned int i = 0; i < size_x * size_y; ++i) {
      if (char_map[i] > max_val) max_val = char_map[i];
      if (char_map[i] >= 254) count_lethal++;
      else if (char_map[i] >= 128) count_inscribed++;
      else if (char_map[i] <= 1) count_free++;
    }
    std::cerr << "[GeometryEngine] Costmap: " << size_x << "x" << size_y 
              << " | MAX=" << (int)max_val
              << " | LETHAL(>=254)=" << count_lethal 
              << " | HIGH(>=128)=" << count_inscribed 
              << " | FREE(<=1)=" << count_free << "\n" << std::flush;
    
    // 二值化: Lower threshold to capture inflated obstacles and bridge gaps
    // Costmap LETHAL=254, INSCRIBED=253. Lowering to 100 includes high-cost areas.
    cv::Mat bin_img;
    cv::threshold(map_img, bin_img, 100, 255, cv::THRESH_BINARY);

    // 膨胀处理 (可选)
    if (inflation_radius_ > 0) {
       double res = costmap->getResolution();
       if (res > 0) {
         int k_size = std::max(1, static_cast<int>(inflation_radius_ / res));
         k_size = std::min(k_size, 10); // 限制最大核大小
         cv::dilate(bin_img, bin_img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_size, k_size)));
       }
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    std::cerr << "[GeometryEngine] 二值化后白色像素: " << cv::countNonZero(bin_img)
              << " | 找到轮廓: " << contours.size() << "\n" << std::flush;

    return contours;
    
  } catch (const std::exception& e) {
    std::cerr << "[GeometryEngine] getContoursFromCostmap 异常: " << e.what() << "\n" << std::flush;
    return {};
  } catch (...) {
    std::cerr << "[GeometryEngine] getContoursFromCostmap 未知异常" << "\n" << std::flush;
    return {};
  }
}


std::vector<Polygon> GeometryEngine::decomposeConvex(
  const std::vector<cv::Point> & contour, 
  double resolution, 
  double origin_x, 
  double origin_y)
{
  // 详细调试：输入验证
  if (contour.empty()) {
    return {};
  }
  
  size_t contour_size = contour.size();
  if (contour_size < 3) {
    return {};
  }
  
  // 检查轮廓点是否有异常值
  int min_x = INT_MAX, max_x = INT_MIN;
  int min_y = INT_MAX, max_y = INT_MIN;
  for (const auto& pt : contour) {
    if (pt.x < min_x) min_x = pt.x;
    if (pt.x > max_x) max_x = pt.x;
    if (pt.y < min_y) min_y = pt.y;
    if (pt.y > max_y) max_y = pt.y;
  }
  
  // 检查坐标是否在合理范围内 (60x60 的 local costmap)
  if (min_x < -1000 || max_x > 10000 || min_y < -1000 || max_y > 10000) {
    std::cerr << "[GeometryEngine] 异常轮廓坐标: x=[" << min_x << "," << max_x 
              << "], y=[" << min_y << "," << max_y << "], 点数=" << contour_size << "\n" << std::flush;
    return {};
  }

  try {
    // 1. 简化轮廓
    std::vector<cv::Point> approx_contour;
    double epsilon_pixels = poly_epsilon_ / resolution;
    if (epsilon_pixels < 0.5) epsilon_pixels = 0.5;  // 最小简化值
    if (epsilon_pixels > 10.0) epsilon_pixels = 10.0; // 最大简化值
    
    cv::approxPolyDP(contour, approx_contour, epsilon_pixels, true);
    
    if (approx_contour.size() < 3) {
      return {};
    }
    
    // 限制简化后的点数
    if (approx_contour.size() > 100) {
      // 太多点，跳过
      std::cerr << "[GeometryEngine] 简化后仍有 " << approx_contour.size() << " 点，跳过" << "\n" << std::flush;
      return {};
    }

    // 2. 计算凸包
    std::vector<cv::Point> hull;
    cv::convexHull(approx_contour, hull, false, true);
    
    if (hull.size() < 3 || hull.size() > 50) {
      std::cerr << "[GeometryEngine] 凸包无效: " << hull.size() << " 点" << "\n" << std::flush;
      return {};
    }

    // 3. 转换为世界坐标
    Polygon poly;
    poly.reserve(hull.size());
    for (const auto& pt : hull) {
      geometry_msgs::msg::Point gpt;
      gpt.x = origin_x + (static_cast<double>(pt.x) + 0.5) * resolution;
      gpt.y = origin_y + (static_cast<double>(pt.y) + 0.5) * resolution;
      gpt.z = 0.0;
      poly.push_back(gpt);
    }
    
    return {poly};
    
  } catch (const cv::Exception& e) {
    std::cerr << "[GeometryEngine] OpenCV 异常: " << e.what() 
              << " | 轮廓点数=" << contour_size 
              << " | 坐标范围: x=[" << min_x << "," << max_x << "], y=[" << min_y << "," << max_y << "]" 
              << "\n" << std::flush;
    return {};
  } catch (const std::exception& e) {
    std::cerr << "[GeometryEngine] decomposeConvex 异常: " << e.what() 
              << " | 轮廓点数=" << contour_size << "\n" << std::flush;
    return {};
  } catch (...) {
    std::cerr << "[GeometryEngine] decomposeConvex 未知异常 | 轮廓点数=" << contour_size << "\n" << std::flush;
    return {};
  }
}




double GeometryEngine::computeDistanceToPath(
  const geometry_msgs::msg::Point & pt, 
  const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  if (path.empty()) return std::numeric_limits<double>::max();

  double min_dist_sq = std::numeric_limits<double>::max();
  
  // 简单的去找最近的路径点
  // 优化：可以只搜索 robot 前方的路径点
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
