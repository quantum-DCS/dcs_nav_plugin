#ifndef DCS_NAV_PLUGIN__HYPERPLANE_PLANNER_HPP_
#define DCS_NAV_PLUGIN__HYPERPLANE_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <opencv2/core.hpp>

namespace dcs_nav_plugin
{

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
  // Internal helpers
  void precomputeObstacles();
  void extractContours(cv::Mat &binary, std::vector<std::vector<cv::Point2f>> &polys_world);
  void convexDecomposeHM(const std::vector<cv::Point2f> &poly,
                         std::vector<std::vector<cv::Point2f>> &convex_parts);
  void mergeConvexByRadius(std::vector<std::vector<cv::Point2f>> &convex_parts, double radius);
  void toHalfspaces(const std::vector<cv::Point2f> &convex,
                    cv::Mat &A, cv::Mat &b);

  bool runAStarPath(const geometry_msgs::msg::PoseStamped &start,
                    const geometry_msgs::msg::PoseStamped &goal,
                    std::vector<cv::Point2f> &path_world);
  void interpolatePath(const std::vector<cv::Point2f> &path_world,
                       size_t N,
                       std::vector<cv::Point2f> &interp);

  // Lifecycle state
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ {nullptr};
  std::string global_frame_;
  std::string name_;

  // Params
  double robot_radius_ {0.1};
  bool use_inflation_layer_ {true};
  int N_ {30};
  int M_ {5};
  double vmax_lin_ {1.0};
  double vmax_ang_ {5.0};
  double amax_lin_ {2.0};
  double amax_ang_ {5.0};
  double solver_time_limit_sec_ {300.0};

  // Precomputed obstacles
  // Each obstacle convex piece: list of vertices (world frame, meters)
  std::vector<std::vector<cv::Point2f>> convex_obstacles_;

  // Cached halfspaces for each convex obstacle
  std::vector<std::pair<cv::Mat, cv::Mat>> obstacle_halfspaces_; // pair(A( m x 2 ), b( m x 1 ))
};

} // namespace dcs_nav_plugin

#endif  // DCS_NAV_PLUGIN__HYPERPLANE_PLANNER_HPP_
