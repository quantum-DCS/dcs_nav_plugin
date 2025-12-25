#ifndef DCS_NAV_PLUGIN__HYPERPLANE_PLANNER_HPP_
#define DCS_NAV_PLUGIN__HYPERPLANE_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Forward declaration for HybridAStar
#include "dcs_nav_plugin/hybrid_a_star.hpp"

// OpenCV for connected components and oriented bounding boxes
#include <opencv2/opencv.hpp>

#ifdef DCS_WITH_CASADI
#include <casadi/casadi.hpp>
#endif

namespace dcs_nav_plugin
{

/**
 * @brief 障碍物表示结构
 * 每个障碍物用半平面表示: A * p <= b
 */
struct Obstacle {
  casadi::DM A;  // n x 2 矩阵，每行是一个半平面的法向量
  casadi::DM b;  // n x 1 向量，每个元素是对应半平面的偏移
  
  Obstacle(const casadi::DM& A_, const casadi::DM& b_) : A(A_), b(b_) {}
};

/**
 * @brief 基于超平面分离的全局规划器（全向轮版本）
 * 
 * 使用 CasADi + IPOPT 进行非线性优化，计算从起点到终点的最优轨迹
 * 使用 HybridAStar 提供 warm start 初始轨迹
 */
class HyperplanePlanner : public nav2_core::GlobalPlanner
{
public:
  HyperplanePlanner();
  ~HyperplanePlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief 从代价地图中提取障碍物
   */
  std::vector<Obstacle> extractObstacles(double min_x, double max_x, double min_y, double max_y);

  /**
   * @brief 根据路径走廊过滤障碍物，只保留靠近路径的障碍物
   * @param obstacles 所有障碍物
   * @param path 参考路径（如HybridA*生成的路径）
   * @param corridor_width 走廊宽度（米）
   * @return 过滤后的障碍物列表
   */
  std::vector<Obstacle> filterObstaclesByPath(
    const std::vector<Obstacle>& obstacles,
    const nav_msgs::msg::Path& path,
    double corridor_width);

  /**
   * @brief 计算车辆顶点（矩形四个角）
   */
  casadi::DM getVehicleVertices(double x, double y, double theta);

  /**
   * @brief 使用 HybridAStar 生成初始轨迹作为 warm start
   * @param start 起点
   * @param goal 终点
   * @param X_init 输出：初始状态轨迹 (6 x N+1)
   * @param U_init 输出：初始控制序列 (3 x N)
   * @return true 成功生成
   */
  bool generateWarmStart(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    casadi::DM& X_init,
    casadi::DM& U_init);

  /**
   * @brief 使用 CasADi 构建并求解优化问题（全向轮模型）
   * @param start_state 起点状态 [x, y, theta, vx, vy, omega]
   * @param goal_state 终点状态 [x, y, theta, vx, vy, omega]
   * @param obstacles 障碍物列表
   * @param X_init 初始状态轨迹 (warm start)
   * @param U_init 初始控制轨迹 (warm start)
   * @param X_out 输出：最优状态轨迹 (6 x N+1)
   * @param U_out 输出：最优控制序列 (3 x N)
   * @param tf_out 输出：最优时间
   * @return true 求解成功
   */
  bool solveOptimization(
    const std::vector<double>& start_state,
    const std::vector<double>& goal_state,
    const std::vector<Obstacle>& obstacles,
    const casadi::DM& X_init,
    const casadi::DM& U_init,
    casadi::DM& X_out,
    casadi::DM& U_out,
    double& tf_out);

  /**
   * @brief 将优化结果转换为 ROS 路径消息
   */
  nav_msgs::msg::Path trajectoryToPath(
    const casadi::DM& X,
    double tf,
    const geometry_msgs::msg::PoseStamped& start);

  // Lifecycle 节点
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;
  std::string global_frame_;
  std::string name_;

  // HybridAStar 用于 warm start
  std::unique_ptr<HybridAStar> hybrid_astar_;

  // 参数：车辆几何
  double vehicle_length_;         // 车辆长度 (m)
  double vehicle_width_;          // 车辆宽度 (m)
  double vehicle_wheelbase_;      // 轴距 (m)
  
  // 参数：优化设置
  int N_;                         // 预测时域长度（离散点数）
  int M_;                         // 每个时间步的积分子步数
  double max_vel_xy_;             // 最大XY速度 (m/s) - 全向轮
  double max_vel_theta_;          // 最大角速度 (rad/s)
  double max_acc_xy_;             // 最大XY加速度 (m/s^2) - 全向轮
  double max_acc_theta_;          // 最大角加速度 (rad/s^2)
  double tf_max_;                 // 最大时间限制 (s)
  
  // 参数：代价函数权重
  double weight_smooth_;          // 平滑性权重
  double weight_time_;            // 时间权重
  
  // 参数：求解器
  double solver_max_iter_;        // IPOPT 最大迭代次数
  double solver_tol_;             // 求解容差
  double solver_time_limit_;      // 求解时间限制 (s)
  
  // 参数：障碍物处理
  double obstacle_inflation_;     // 障碍物膨胀距离 (m)
  double lambda_max_;             // 超平面参数上限
  double mu_max_;                 // 超平面参数上限
  double eps_;                    // 数值稳定性小量
  
  // 参数：边界约束
  casadi::DM A_canvas_;           // 画布/边界约束矩阵
  casadi::DM b_canvas_;           // 画布/边界约束向量

  static constexpr double PI = 3.14159265358979323846;
};

}  // namespace dcs_nav_plugin

#endif  // DCS_NAV_PLUGIN__HYPERPLANE_PLANNER_HPP_
