#include "dcs_nav_plugin/debug_utils.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace dcs_nav_plugin
{

DebugUtils::DebugUtils() {}

DebugUtils::~DebugUtils()
{
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

std::string DebugUtils::getTimestampString()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}

std::string getPreciseTimestampString()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%H:%M:%S"); // Local time HH:MM:SS
  ss << "." << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

void DebugUtils::createDirectory(const std::string& path)
{
    // Use system command for simplicity to create recursive directories
    std::string cmd = "mkdir -p " + path;
    int ret = system(cmd.c_str());
    (void)ret;
}

std::string DebugUtils::expandUserPath(const std::string& path)
{
    if (path.empty()) return path;
    
    // Check if starts with "~/"
    if (path.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home) {
            return std::string(home) + path.substr(1);
        }
    }
    return path;
}

void DebugUtils::initialize(const std::string& path, const std::string& node_name)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (initialized_) return;

    std::string full_path = expandUserPath(path);
    if (full_path.empty()) {
        full_path = "/tmp/planner_log"; // Safe fallback
    }
    
    log_dir_ = full_path;
    
    // 1. Ensure directory exists
    createDirectory(log_dir_);
    
    // 2. Clear previous logs -> REMOVED to avoid race condition between Planner and Controller
    // Cleaming should be done by launch script.
    // if (log_dir_.size() > 5) {
    //    std::string cmd = "rm -rf " + log_dir_ + "/*";
    //    ...
    // }

    // 3. Create Log File with Timestamp and Node Name
    std::string timestamp = getTimestampString();
    // e.g. "log_2025-01-01_12-00-00_controller.txt"
    std::string log_filename = "log_" + timestamp + "_" + node_name + ".txt";
    std::string log_path = log_dir_ + "/" + log_filename;
    
    log_file_.open(log_path, std::ios::out | std::ios::app);
    
    if (log_file_.is_open()) {
        std::cout << "[DebugUtils] Log file created at: " << log_path << std::endl;
        log_file_ << "=== Optimization & Planner Log Started at " << timestamp << " ===" << std::endl;
    } else {
        std::cerr << "[DebugUtils] Failed to create log file at: " << log_path << std::endl;
    }

    initialized_ = true;
}

void DebugUtils::ensureInitialized()
{
    if (!initialized_) {
        initialize("/tmp/planner_log_fallback"); 
    }
}

// 内部辅助写入函数
void writeLog(std::ofstream& file, const std::string& msg, bool to_console) {
    std::string timestamp = getPreciseTimestampString();
    std::string line = "[" + timestamp + "] " + msg;
    
    if (to_console) {
        std::cout << line << std::endl;
    }
    if (file.is_open()) {
        file << line << std::endl;
        file.flush();
    }
}

void DebugUtils::log(const std::string& msg)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    writeLog(log_file_, msg, true);
}

void DebugUtils::logToFile(const std::string& msg)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    writeLog(log_file_, msg, false);
}

// ------ Image Saving Functions ------

// Helper: Convert Costmap2D to cv::Mat
cv::Mat costmapToMat(nav2_costmap_2d::Costmap2D* costmap)
{
    unsigned int nx = costmap->getSizeInCellsX();
    unsigned int ny = costmap->getSizeInCellsY();
    unsigned char* char_map = costmap->getCharMap();

    // Create Mat directly from data (CV_8UC1)
    // Note: Costmap data is row-major, origin at (0,0) usually bottom-left.
    // OpenCV Mat is also row-major.
    // BUT we need to be careful about orientation. cv::Mat (0,0) is top-left.
    // Costmap (0,0) is usually bottom-left in world coords.
    // We will just copy data first.
    
    cv::Mat map_img(ny, nx, CV_8UC1, char_map);
    
    // Costmap values: 0 (free) to 254 (lethal), 255 (unknown)
    // For visualization: 
    // 0 -> 255 (White)
    // 254 -> 0 (Black)
    // 255 -> 128 (Gray)
    
    cv::Mat vis_img(ny, nx, CV_8UC3);
    
    for(unsigned int y=0; y<ny; ++y) {
        for(unsigned int x=0; x<nx; ++x) {
            unsigned char val = map_img.at<unsigned char>(y, x);
            cv::Vec3b color;
            if (val == 255) {
                color = cv::Vec3b(128, 128, 128); // Unknown = Gray
            } else if (val == 0) {
                color = cv::Vec3b(255, 255, 255); // Free = White
            } else if (val >= 253) {
                 color = cv::Vec3b(0, 0, 0);       // Lethal = Black
            } else {
                // Scaled Obstacles: Darker as cost increases
                // val is 1..252. Map 1->250 (light) to 252->0 (dark)
                 unsigned char v = static_cast<unsigned char>(255 - val);
                 color = cv::Vec3b(v, v, v);
            }
            vis_img.at<cv::Vec3b>(ny - 1 - y, x) = color; // Flip Y to match standard image coords (Top-Left 0,0) vs Costmap (Bottom-Left 0,0)
        }
    }
    return vis_img;
}

void DebugUtils::saveLocalCostmap(nav2_costmap_2d::Costmap2D* costmap, int frame_id)
{
    ensureInitialized();
    cv::Mat img = costmapToMat(costmap);
    
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/%04d_1_costmap.png", log_dir_.c_str(), frame_id);
    cv::imwrite(filename, img);
}

void DebugUtils::saveCostmapWithPolygons(
    nav2_costmap_2d::Costmap2D* costmap, 
    const std::vector<Polygon>& polygons,
    const std::vector<Polygon>& selected_polygons,
    int frame_id)
{
    ensureInitialized();
    cv::Mat img = costmapToMat(costmap);
    
    double res = costmap->getResolution();
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    int size_y = costmap->getSizeInCellsY();
    
    // Transform World -> Image Pixel
    auto worldToImg = [&](double wx, double wy) -> cv::Point {
        // MX = (wx - ox) / res
        // MY = (wy - oy) / res
        int mx = static_cast<int>((wx - origin_x) / res);
        int my = static_cast<int>((wy - origin_y) / res);
        
        // Flip Y for image
        return cv::Point(mx, size_y - 1 - my);
    };

    // Draw ALL polygons (Blue)
    drawPolygons(img, polygons, cv::Scalar(255, 0, 0), worldToImg);
    
    // Draw SELECTED polygons (Red, Thicker)
    drawPolygons(img, selected_polygons, cv::Scalar(0, 0, 255), worldToImg);
    
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/%04d_2_polygons.png", log_dir_.c_str(), frame_id);
    cv::imwrite(filename, img);
}

void DebugUtils::saveGlobalView(
    const nav_msgs::msg::OccupancyGrid::SharedPtr global_map,
    const std::vector<geometry_msgs::msg::PoseStamped>& global_path,
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path,
    const geometry_msgs::msg::PoseStamped& current_pose,
    const std::vector<Polygon>& polygons,
    int frame_id
)
{
    ensureInitialized();
    if (!global_map) return;

    // 1. Create Global Map Image
    // 放大倍数
    const int SCALE = 2;

    int width = global_map->info.width * SCALE;
    int height = global_map->info.height * SCALE;
    double res = global_map->info.resolution / SCALE; // Effective pixel resolution is smaller (finer)
    double ox = global_map->info.origin.position.x;
    double oy = global_map->info.origin.position.y;
    
    // Create base image from occupancy grid
    // For large maps, this might be slow, but it's for debug.
    // Optimization: Keep a persistent base image and only copy it.
    // For now, reconstruct.
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(128, 128, 128)); // Default Gray
    
    int map_w = global_map->info.width;
    int map_h = global_map->info.height;

    for(int y=0; y<map_h; ++y) {
        for(int x=0; x<map_w; ++x) {
            int idx = y * map_w + x;
            int8_t val = global_map->data[idx];
            cv::Vec3b color;
            if (val == -1) color = cv::Vec3b(128, 128, 128);
            else if (val == 0) color = cv::Vec3b(255, 255, 255);
            else if (val == 100) color = cv::Vec3b(0, 0, 0);
            else {
                 unsigned char v = static_cast<unsigned char>(255 - (val * 255 / 100));
                 color = cv::Vec3b(v, v, v);
            }
            
            // Fill SCALE x SCALE block
            for(int dy=0; dy<SCALE; ++dy) {
                for(int dx=0; dx<SCALE; ++dx) {
                     // Check bounds
                     int img_y = (height - 1) - (y * SCALE + dy); // Flip Y at image level. 
                     // Wait, let's keep it simple: map(x,y) -> image(x*S, (h-1-y)*S) block?
                     // Standard Flip: Image(0,0) is Top-Left. Map(0,0) is Bottom-Left.
                     // Map Y=0 => Image Y=H-1...H-S
                     // Map Y=k => Image Y range [H - (k+1)*S, H - k*S - 1]
                     
                     int im_x = x * SCALE + dx;
                     int im_y = (map_h - 1 - y) * SCALE + dy; 
                     // Let's re-verify Flip logic. 
                     // Map (0,0) -> Image (0, H-S..H-1)
                     // Map (0, 1) -> Image (0, H-2S..H-S-1)
                     
                     if (im_x >=0 && im_x < width && im_y >=0 && im_y < height)
                        img.at<cv::Vec3b>(im_y, im_x) = color;
                }
            }
        }
    }
    
    // Coordinate Transform
    auto worldToImg = [&](double wx, double wy) -> cv::Point {
        // (wx - ox) / map_res gives cell float coordinate.
        // Multiply by SCALE to get image coordinate.
        double map_cell_x = (wx - ox) / global_map->info.resolution;
        double map_cell_y = (wy - oy) / global_map->info.resolution;
        
        int mx = static_cast<int>(map_cell_x * SCALE);
        // Correct Y-flip for Point:
        // map_cell_y=0 => image_y should be near Height-1
        // map_cell_y=MH => image_y near 0
        int my = static_cast<int>((map_h - map_cell_y) * SCALE) - 1; 
        // Or simpler: Height - 1 - (int)(map_cell_y * SCALE) ?
        // Let's stick to the block fill logic:
        // cell(x,y) corresponds to image rect: x*S to (x+1)*S, (MH-1-y)*S to (MH-y)*S
        // Center of cell(x,y) is x+0.5, y+0.5
        // Image Y = (MH - (y+0.5)) * SCALE
        
        int im_y = static_cast<int>((map_h * SCALE) - 1 - (map_cell_y * SCALE));

        return cv::Point(mx, im_y);
    };
    
    // 2. Draw Global Path (Green)
    drawPath(img, global_path, cv::Scalar(0, 255, 0), worldToImg);
    
    // 3. Draw Local Path / MPC Predicted (Cyan)
    drawPath(img, local_path, cv::Scalar(255, 255, 0), worldToImg);
    
    // 4. Draw Polygons (Red)
    drawPolygons(img, polygons, cv::Scalar(0, 0, 255), worldToImg);
    
    // 5. Draw Robot Pose (Blue Circle)
    cv::Point robot_pt = worldToImg(current_pose.pose.position.x, current_pose.pose.position.y);
    cv::circle(img, robot_pt, 6, cv::Scalar(255, 0, 0), -1); // Slightly larger
    
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/%04d_3_global.png", log_dir_.c_str(), frame_id);
    cv::imwrite(filename, img);
}

void DebugUtils::drawPath(cv::Mat& img, 
              const std::vector<geometry_msgs::msg::PoseStamped>& path, 
              const cv::Scalar& color,
              std::function<cv::Point(double, double)> transform_func)
{
    if (path.empty()) return;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::Point p1 = transform_func(path[i].pose.position.x, path[i].pose.position.y);
        cv::Point p2 = transform_func(path[i+1].pose.position.x, path[i+1].pose.position.y);
        // THICKER LINE
        cv::line(img, p1, p2, color, 3); 
    }
}

void DebugUtils::drawPolygons(cv::Mat& img,
                  const std::vector<Polygon>& polys,
                  const cv::Scalar& color,
                  std::function<cv::Point(double, double)> transform_func)
{
    for(const auto& poly : polys) {
        if (poly.size() < 2) continue;
        std::vector<cv::Point> pts;
        for(const auto& pt : poly) {
            pts.push_back(transform_func(pt.x, pt.y));
        }
        
        // Draw open or closed? Obstacles are closed.
        const cv::Point* ppt[1] = { &pts[0] };
        int npt[] = { (int)pts.size() };
        // THICKER LINE
        cv::polylines(img, ppt, npt, 1, true, color, 2);
    }
}

} // namespace dcs_nav_plugin
