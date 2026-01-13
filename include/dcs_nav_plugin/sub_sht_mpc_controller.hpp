#ifndef DCS_NAV_PLUGIN__SUB_SHT_MPC_CONTROLLER_HPP_
#define DCS_NAV_PLUGIN__SUB_SHT_MPC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "dcs_nav_plugin/geometry/geometry_engine.hpp"
#include "dcs_nav_plugin/mpc/mpc_solver_casadi.hpp"
#include "dcs_nav_plugin/debug_utils.hpp"
#include "rcl_interfaces/msg/log.hpp"

namespace dcs_nav_plugin
{

class DcsShtMpcController : public nav2_core::Controller
{
public:
  DcsShtMpcController() = default;
  ~DcsShtMpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    std::string name, 
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

private:
  // Components
  std::shared_ptr<GeometryEngine> geometry_engine_;
  std::shared_ptr<MpcSolverCasadi> mpc_solver_;
  
  // ROS Interfaces
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DcsShtMpcController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  
  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_traj_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr all_obs_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr sel_obs_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  // Data
  nav_msgs::msg::Path global_plan_;
  std::string plugin_name_;
  
  // Parameters
  MpcConfig config_;
  
  // Geometry Engine Parameters
  double geometry_poly_epsilon_{0.05};
  double geometry_inflation_radius_{0.0};
  int geometry_top_k_{6};
  int geometry_dilation_pixels_{1};
  
  // Omni Directional
  bool enable_omni_movement_{true};
  
  // Rotation Correction (for Gazebo simulation issue)
  bool invert_rotation_{false};
  
  // Helpers
  void loadParameters();
  void publishVisualizations(
      const std::vector<Polygon> & all_polygons,
      const std::vector<Polygon> & selected_polygons,
      const std::vector<geometry_msgs::msg::PoseStamped> & ref_traj,
      const std::vector<geometry_msgs::msg::PoseStamped> & mpc_traj);
      
  std::vector<geometry_msgs::msg::PoseStamped> generateReferenceTrajectory(
      const geometry_msgs::msg::PoseStamped & current_pose,
      const nav_msgs::msg::Path & plan,
      int N, double dt);

  bool checkCollision(
      const std::vector<geometry_msgs::msg::PoseStamped> & trajectory,
      const std::vector<Polygon> & obstacles);

  bool transformPlan(
      const nav_msgs::msg::Path & plan,
      const geometry_msgs::msg::PoseStamped & pose,
      nav_msgs::msg::Path & transformed_plan);

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Debug Utils
  std::shared_ptr<DebugUtils> debug_utils_;
  std::string debug_log_dir_;
  
  // Global Map Subscription for visualization
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_global_map_;
  
  int debug_frame_count_ = 0;
};

}  // namespace dcs_nav_plugin

#endif  // DCS_NAV_PLUGIN__SUB_SHT_MPC_CONTROLLER_HPP_
