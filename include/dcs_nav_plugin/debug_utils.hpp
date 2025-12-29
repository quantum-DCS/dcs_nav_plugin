#ifndef DCS_NAV_PLUGIN__DEBUG_UTILS_HPP_
#define DCS_NAV_PLUGIN__DEBUG_UTILS_HPP_

#include <string>
#include <fstream>
#include <vector>
#include <mutex>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace dcs_nav_plugin
{

class DebugUtils
{
public:
  DebugUtils();
  ~DebugUtils();

  // 初始化：指定日志路径 (支持 ~/ 路径展开) 和 节点名称 (用于区分日志文件)
  void initialize(const std::string& path, const std::string& node_name = "node");

  // 若未初始化，则先调用初始化 (使用空路径，这将触发默认行为或报错)
  // 为了安全，ensureInitialized 应该依赖于已经 configure 过的状态。
  // 但我们已经移除了默认参数，所以这里应该不做更改，或者我们可以在 initialize 中给个默认参数?
  // 按照计划，控制器一定会传参。
  void ensureInitialized();

  // 双路日志输出：终端 + 文件 (带时间戳)
  void log(const std::string& msg);
  
  // 仅输出到文件 (带时间戳, 用于保存 /rosout 捕获的日志避免重复打印)
  void logToFile(const std::string& msg);

  // 保存局部代价地图 (Image 1)
  void saveLocalCostmap(nav2_costmap_2d::Costmap2D* costmap, int frame_id);

  // 保存局部代价地图 + 障碍物多边形 (Image 2)
  // 使用 geometry_msgs::msg::Point 以匹配 GeometryEngine::Polygon
  using Polygon = std::vector<geometry_msgs::msg::Point>;
  
  void saveCostmapWithPolygons(
    nav2_costmap_2d::Costmap2D* costmap, 
    const std::vector<Polygon>& polygons,
    const std::vector<Polygon>& selected_polygons,
    int frame_id);

  // 保存全局视角图 (Image 3)
  // 参数：
  // global_map: 静态全局地图
  // global_path: 全局规划器生成的路径
  // local_path: MPC 预测的局部路径
  // costmap_pose: 局部代价地图中心在全局坐标系下的位姿
  // costmap_size_x/y: 局部代价地图的物理尺寸 (米)
  // polygons: 障碍物多边形 (全局坐标系或局部坐标系? 通常 geometry_engine 输出是在 global frame)
  void saveGlobalView(
    const nav_msgs::msg::OccupancyGrid::SharedPtr global_map,
    const std::vector<geometry_msgs::msg::PoseStamped>& global_path,
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path,
    const geometry_msgs::msg::PoseStamped& current_pose, // 机器人当前位姿
    const std::vector<Polygon>& polygons,
    int frame_id
  );

private:
  std::string getTimestampString();
  void createDirectory(const std::string& path);
  std::string expandUserPath(const std::string& path);
  
  // Helper to draw path on CV image
  // transform_func: 将世界坐标 (x,y) 转换为图像像素坐标 (u,v)
  void drawPath(cv::Mat& img, 
                const std::vector<geometry_msgs::msg::PoseStamped>& path, 
                const cv::Scalar& color,
                std::function<cv::Point(double, double)> transform_func);

  void drawPolygons(cv::Mat& img,
                    const std::vector<Polygon>& polys,
                    const cv::Scalar& color,
                    std::function<cv::Point(double, double)> transform_func);

  std::string log_dir_;
  std::ofstream log_file_;
  bool initialized_ = false;
  std::mutex log_mutex_;
};

} // namespace dcs_nav_plugin

#endif // DCS_NAV_PLUGIN__DEBUG_UTILS_HPP_
