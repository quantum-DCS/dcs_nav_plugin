#ifndef DCS_NAV_PLUGIN__GEOMETRY__GEOMETRY_ENGINE_HPP_
#define DCS_NAV_PLUGIN__GEOMETRY__GEOMETRY_ENGINE_HPP_

#include <vector>
#include <memory>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "polypartition.h"

namespace dcs_nav_plugin
{

// 表示一个多边形 (一组有序的点)
using Polygon = std::vector<geometry_msgs::msg::Point>;

struct Obstacle
{
  Polygon polygon;
  geometry_msgs::msg::Point centroid;
  double dist_to_path;
};

/**
 * @class GeometryEngine
 * @brief 负责从 Costmap 提取多边形并进行凸分解处理
 */
class GeometryEngine
{
public:
  GeometryEngine();
  ~GeometryEngine();

  /**
   * @brief 配置参数
   */
  void configure(
    double poly_epsilon, 
    double obstacle_height_thres,
    double inflation_radius,
    int top_k,
    int dilation_pixels);

  /**
   * @brief 从 Costmap 中提取障碍物，并返回经过凸分解和筛选后的多边形列表
   * @param costmap 指向局部代价地图的指针
   * @param robot_pose 机器人当前位姿
   * @param ref_path 参考路径 (用于计算 Top-K 距离)
   * @param all_polygons [Out] 输出所有提取到的凸多边形 (用于调试可视化)
   * @return 最终筛选出的 Top-K 凸多边形列表 (用于 MPC)
   */
  std::vector<Polygon> extractObstacles(
    nav2_costmap_2d::Costmap2D * costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const std::vector<geometry_msgs::msg::PoseStamped> & ref_path,
    std::vector<Polygon> & all_polygons_out);

private:
  // 参数
  double poly_epsilon_;      // 多边形近似精度 (m)
  double min_obstacle_area_; // 最小障碍物面积过滤
  double inflation_radius_;  // 预处理膨胀半径
  int top_k_;                // 保留前 K 个
  int dilation_pixels_;      // 膨胀像素数 (0=禁用)

  // Helper functions
  std::vector<std::vector<cv::Point>> getContoursFromCostmap(nav2_costmap_2d::Costmap2D * costmap);
  std::vector<Polygon> decomposeConvex(const std::vector<cv::Point> & contour, double resolution, double origin_x, double origin_y);
  double computeDistanceToPath(const geometry_msgs::msg::Point & pt, const std::vector<geometry_msgs::msg::PoseStamped> & path);
  
  // Debug log file
  std::ofstream debug_log_;
};

}  // namespace dcs_nav_plugin

#endif  // DCS_NAV_PLUGIN__GEOMETRY__GEOMETRY_ENGINE_HPP_
