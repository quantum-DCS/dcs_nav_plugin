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
 * @brief 基于超平面分离的全局规划器
 * 
 * 使用 CasADi + IPOPT 进行非线性优化，计算从起点到终点的最优轨迹
 * 核心思想：利用分离超平面定理确保车辆与障碍物不相撞
 */
class HyperplanePlanner : public nav2_core::GlobalPlanner
{
public:
  HyperplanePlanner();
  ~HyperplanePlanner() override;

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
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
   * @return 障碍物列表（每个障碍物用半平面 A*p <= b 表示）
   */
  std::vector<Obstacle> extractObstacles();

  /**
   * @brief 计算车辆顶点（矩形四个角）
   * @param x 车辆中心 x 坐标
   * @param y 车辆中心 y 坐标
   * @param theta 车辆朝向角
   * @return 4x2 矩阵，每列是一个顶点 [x; y]
   */
  casadi::DM getVehicleVertices(double x, double y, double theta);

  /**
   * @brief 使用 CasADi 构建并求解优化问题
   * @param start_state 起点状态 [x, y, theta, v, omega]
   * @param goal_state 终点状态 [x, y, theta, v, omega]
   * @param obstacles 障碍物列表
   * @param X_out 输出：最优状态轨迹 (5 x N+1)
   * @param U_out 输出：最优控制序列 (2 x N)
   * @param tf_out 输出：最优时间
   * @return true 求解成功
   */
  bool solveOptimization(
    const std::vector<double>& start_state,
    const std::vector<double>& goal_state,
    const std::vector<Obstacle>& obstacles,
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

  // 参数：车辆几何
  double vehicle_length_;         // 车辆长度 (m)
  double vehicle_width_;          // 车辆宽度 (m)
  double vehicle_wheelbase_;      // 轴距 (m)
  
  // 参数：优化设置
  int N_;                         // 预测时域长度（离散点数）
  int M_;                         // 每个时间步的积分子步数
  double max_vel_x_;              // 最大线速度 (m/s)
  double max_vel_theta_;          // 最大角速度 (rad/s)
  double max_acc_x_;              // 最大线加速度 (m/s^2)
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
