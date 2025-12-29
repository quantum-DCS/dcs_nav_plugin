#ifndef DCS_NAV_PLUGIN__HYBRID_A_STAR_HPP_
#define DCS_NAV_PLUGIN__HYBRID_A_STAR_HPP_

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "dcs_nav_plugin/debug_utils.hpp"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>

namespace dcs_nav_plugin
{

struct Node3D {
  double x, y, theta;
  double g, h;
  double cost;
  int prim; // primitive index used to get here
  Node3D* parent;
  bool is_reverse; // true if the motion to get here was reverse

  Node3D(double x = 0, double y = 0, double theta = 0, 
         double g = 0, double h = 0, Node3D* parent = nullptr, int prim = 0, bool is_reverse = false)
      : x(x), y(y), theta(theta), g(g), h(h), cost(g + h), prim(prim), parent(parent), is_reverse(is_reverse) {}

  // Grid coordinates for indexing
  int x_idx, y_idx, theta_idx;
  
  void setIndex(int xi, int yi, int thi) {
    x_idx = xi;
    y_idx = yi;
    theta_idx = thi;
  }
  
  // For priority queue
  bool operator>(const Node3D& other) const {
    return cost > other.cost;
  }
};

struct Node3DComparator {
  bool operator()(const Node3D* a, const Node3D* b) const {
    return a->cost > b->cost;
  }
};

class HybridAStar : public nav2_core::GlobalPlanner
{
public:
  HybridAStar();
  ~HybridAStar();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // Helper methods
  bool isCollision(double x, double y, double theta);
  bool isCollision(const Node3D& node);
  double getHeuristic(const Node3D& node, const Node3D& goal);
  std::vector<Node3D*> getNeighbors(Node3D* current, const Node3D& goal);
  std::vector<Node3D*> getNeighborsAckermann(Node3D* current, const Node3D& goal);
  std::vector<Node3D*> getNeighborsOmnidirectional(Node3D* current, const Node3D& goal);
  bool getIndex(const Node3D& node, int& x_idx, int& y_idx, int& theta_idx);
  nav_msgs::msg::Path reconstructPath(Node3D* node, const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal);
  nav_msgs::msg::Path tryAnalyticExpansion(Node3D* current, const Node3D& goal);
  nav_msgs::msg::Path tryLinearExpansion(Node3D* current, const Node3D& goal);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  
  // Parameters
  // Motion model selection
  std::string motion_model_;  // "ACKERMANN" or "OMNIDIRECTIONAL"
  
  // Common parameters
  double interpolation_resolution_;
  double shot_distance_; // Distance to try analytic expansion (Reeds-Shepp/Dubins)
  int theta_discretization_; // Number of headings
  
  // Ackermann-specific parameters
  double min_turning_radius_;
  double reverse_penalty_;
  double direction_change_penalty_;
  
  // Omnidirectional-specific parameters
  double lateral_step_size_;    // Lateral movement step size
  double rotation_step_;        // In-place rotation step (radians)
  double rotation_penalty_;     // Cost multiplier for rotation
  double diagonal_penalty_;     // Cost multiplier for diagonal movement
  
  // OMPL Reeds-Shepp Space
  std::shared_ptr<ompl::base::ReedsSheppStateSpace> reeds_shepp_space_;

  // Constants
  static constexpr double PI = 3.14159265359;
  
  // Debug
  std::shared_ptr<DebugUtils> debug_utils_;
};

}  // namespace dcs_nav_plugin

#endif  // DCS_NAV_PLUGIN__HYBRID_A_STAR_HPP_
