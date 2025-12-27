#include "dcs_nav_plugin/sub_sht_mpc_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <tf2/utils.h>

namespace dcs_nav_plugin
{

void DcsShtMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  clock_ = parent.lock()->get_clock();
  logger_ = parent.lock()->get_logger();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  plugin_name_ = name;
  tf_buffer_ = tf;

  // load parameters
  loadParameters();

  // Components
  geometry_engine_ = std::make_shared<GeometryEngine>();
  geometry_engine_->configure(0.05, 0.5, 0.0, 6); // Load from params actually

  mpc_solver_ = std::make_shared<MpcSolverCasadi>();
  mpc_solver_->configure(config_);
  mpc_solver_->initialize();

  // Publishers
  auto node = parent.lock();
  all_obs_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name + "/all_obstacles", 1);
  sel_obs_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name + "/selected_obstacles", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/local_plan", 1);
  
  RCLCPP_INFO(logger_, "Configured DcsShtMpcController: %s", name.c_str());
}

void DcsShtMpcController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up DcsShtMpcController");
  // Release resources
  all_obs_pub_.reset();
  sel_obs_pub_.reset();
  local_plan_pub_.reset();
}

void DcsShtMpcController::activate()
{
  RCLCPP_INFO(logger_, "Activating DcsShtMpcController");
  all_obs_pub_->on_activate();
  sel_obs_pub_->on_activate();
  local_plan_pub_->on_activate();
}

void DcsShtMpcController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating DcsShtMpcController");
  all_obs_pub_->on_deactivate();
  sel_obs_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
}

void DcsShtMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (!path.poses.empty()) {
    RCLCPP_INFO(logger_, "[SHT-MPC] 收到新全局路径: %zu 点, 终点: (%.2f, %.2f)", 
      path.poses.size(), 
      path.poses.back().pose.position.x, 
      path.poses.back().pose.position.y);
  } else {
    RCLCPP_WARN(logger_, "[SHT-MPC] 收到空路径!");
  }
}

void DcsShtMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Implementation for speed limit override
}

void DcsShtMpcController::loadParameters()
{
  auto node = parent_.lock();
  
  // Helper macro for cleaner code
  auto declare_and_get = [&](const std::string& param, auto& target, auto default_val) {
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + param, rclcpp::ParameterValue(default_val));
    node->get_parameter(plugin_name_ + param, target);
  };
  
  // MPC Parameters
  declare_and_get(".mpc.N", config_.N, 20);
  declare_and_get(".mpc.dt", config_.dt, 0.1);
  declare_and_get(".mpc.v_max", config_.v_max, 0.3);
  declare_and_get(".mpc.omega_max", config_.omega_max, 0.5);
  declare_and_get(".mpc.Q_x", config_.Q_x, 10.0);
  declare_and_get(".mpc.Q_y", config_.Q_y, 10.0);
  declare_and_get(".mpc.Q_theta", config_.Q_theta, 10.0);
  declare_and_get(".mpc.R_v", config_.R_v, 0.1);
  declare_and_get(".mpc.R_omega", config_.R_omega, 10.0);
  declare_and_get(".mpc.lambda_slack", config_.lambda_slack, 100000.0);
  declare_and_get(".mpc.margin", config_.margin, 0.15);
  declare_and_get(".mpc.sht_epsilon", config_.sht_epsilon, 0.001);
  declare_and_get(".mpc.car_width", config_.car_width, 0.3);
  declare_and_get(".mpc.car_length", config_.car_length, 0.3);
  
  // Log loaded parameters
  RCLCPP_INFO(logger_, "[SHT-MPC] 参数已加载: v_max=%.2f, omega_max=%.2f, Q_theta=%.1f, R_omega=%.1f",
              config_.v_max, config_.omega_max, config_.Q_theta, config_.R_omega);
}

geometry_msgs::msg::TwistStamped DcsShtMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker; // unused

  // 1. Transform Global Plan to Controller Frame (Costmap Frame)
  nav_msgs::msg::Path transformed_plan;
  if (!transformPlan(global_plan_, pose, transformed_plan))
  {
      RCLCPP_WARN(logger_, "Could not transform the global plan to the frame of the controller");
      return geometry_msgs::msg::TwistStamped();
  }
  
  if (!transformed_plan.poses.empty()) {
      double dist_to_goal = std::hypot(
        transformed_plan.poses.back().pose.position.x - pose.pose.position.x,
        transformed_plan.poses.back().pose.position.y - pose.pose.position.y
      );
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
        "[SHT-MPC] 距离目标: %.2fm (变换后终点: %.2f, %.2f) (当前: %.2f, %.2f)", 
        dist_to_goal,
        transformed_plan.poses.back().pose.position.x, transformed_plan.poses.back().pose.position.y,
        pose.pose.position.x, pose.pose.position.y);
  }

  // 2. Generate Reference Trajectory (using transformed plan)
  auto ref_traj = generateReferenceTrajectory(pose, transformed_plan, config_.N, config_.dt);
  
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, 
    "[SHT-MPC] 参考轨迹: %zu 点, 当前位置: (%.2f, %.2f)", 
    ref_traj.size(), pose.pose.position.x, pose.pose.position.y);

  // 2. Extract Obstacles
  std::vector<Polygon> all_polys;
  auto selected_polys = geometry_engine_->extractObstacles(
      costmap_, pose, ref_traj, all_polys
  );
  
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
    "[SHT-MPC] 障碍物: 全部=%zu, 筛选=%zu", 
    all_polys.size(), selected_polys.size());

  // 3. Solve MPC
  auto sol = mpc_solver_->solve(pose, velocity, ref_traj, selected_polys);
  
  // Log solve result
  if (sol.success) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
      "[SHT-MPC] ✓ 求解成功 (%.1f ms) | 速度: vx=%.2f, vy=%.2f, ω=%.2f | 松弛=%.4f", 
      sol.solve_time_ms, 
      sol.cmd_vel.linear.x, sol.cmd_vel.linear.y, sol.cmd_vel.angular.z,
      sol.max_slack);
  } else {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
      "[SHT-MPC] ✗ 求解失败 (%.1f ms) | 检查日志获取详情", 
      sol.solve_time_ms);
  }
  
  // 3.1 Post-Solve Safety Shield
  if (sol.success) {
      // Check full collision against ALL polys (not just selected)
      if (checkCollision(sol.predicted_traj, all_polys)) {
          RCLCPP_ERROR(logger_, "[SHT-MPC] 安全防线: 预测轨迹碰撞! 紧急制动。");
          sol.success = false;
          sol.cmd_vel = geometry_msgs::msg::Twist(); // Zero
      }
  }
  
  // 4. Publish Visualization
  publishVisualizations(all_polys, selected_polys, ref_traj, sol.predicted_traj);

  // 5. Output Command
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = "base_link"; // Control output commonly in base_link

  if (sol.success) {
      // Check Slack - 阈值 1.0 表示约束被显著违反
      if (sol.max_slack > 1.0) {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, 
            "[SHT-MPC] 高松弛 (%.2f)! 约束被违反，可能接近障碍物。", sol.max_slack);
      }
      
      cmd.twist = sol.cmd_vel; 
  } else {
      cmd.twist.linear.x = 0;
      cmd.twist.linear.y = 0;
      cmd.twist.angular.z = 0;
  }
  
  return cmd;
}

std::vector<geometry_msgs::msg::PoseStamped> DcsShtMpcController::generateReferenceTrajectory(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & plan,
    int N, double dt)
{
    std::vector<geometry_msgs::msg::PoseStamped> ref_traj;
    
    if (plan.poses.empty()) {
        for(int i=0; i<=N; ++i) ref_traj.push_back(current_pose);
        return ref_traj;
    }

    // 获取当前位置
    double current_x = current_pose.pose.position.x;
    double current_y = current_pose.pose.position.y;
    
    // 1. 找到路径上距离当前位置最近的点索引
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < plan.poses.size(); ++i) {
        double dx = plan.poses[i].pose.position.x - current_x;
        double dy = plan.poses[i].pose.position.y - current_y;
        double d2 = dx*dx + dy*dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            closest_idx = i;
        }
    }
    
    // 2. 计算从 closest_idx 开始的累计弧长
    std::vector<double> arc_lengths;
    arc_lengths.push_back(0.0);
    double total_arc = 0.0;
    
    for (size_t i = closest_idx + 1; i < plan.poses.size(); ++i) {
        double dx = plan.poses[i].pose.position.x - plan.poses[i-1].pose.position.x;
        double dy = plan.poses[i].pose.position.y - plan.poses[i-1].pose.position.y;
        total_arc += std::hypot(dx, dy);
        arc_lengths.push_back(total_arc);
    }
    
    // 3. 计算 MPC 预测时域内的预视距离
    double v_ref = config_.v_max;
    if (v_ref < 0.1) v_ref = 0.3;
    double lookahead_dist = N * v_ref * dt;  // MPC时域内能走的距离
    double sample_dist = std::min(lookahead_dist, total_arc);
    
    // 4. 沿路径均匀采样 N+1 个点
    for (int k = 0; k <= N; ++k) {
        double target_arc = (static_cast<double>(k) / static_cast<double>(N)) * sample_dist;
        
        // 在 arc_lengths 中二分查找
        size_t seg_idx = 0;
        for (size_t i = 1; i < arc_lengths.size(); ++i) {
            if (arc_lengths[i] >= target_arc) {
                seg_idx = i - 1;
                break;
            }
            seg_idx = i;  // target_arc 超过了总长度，使用最后一段
        }
        
        // 在这一段内插值
        size_t path_idx = closest_idx + seg_idx;
        size_t next_idx = std::min(path_idx + 1, plan.poses.size() - 1);
        
        double seg_start_arc = arc_lengths[seg_idx];
        double seg_end_arc = (seg_idx + 1 < arc_lengths.size()) ? arc_lengths[seg_idx + 1] : arc_lengths.back();
        double seg_len = seg_end_arc - seg_start_arc;
        
        double t = 0.0;
        if (seg_len > 0.001) {
            t = (target_arc - seg_start_arc) / seg_len;
            t = std::clamp(t, 0.0, 1.0);
        }
        
        geometry_msgs::msg::PoseStamped p;
        p.header = current_pose.header;
        
        // 线性插值位置
        p.pose.position.x = (1.0 - t) * plan.poses[path_idx].pose.position.x + 
                            t * plan.poses[next_idx].pose.position.x;
        p.pose.position.y = (1.0 - t) * plan.poses[path_idx].pose.position.y + 
                            t * plan.poses[next_idx].pose.position.y;
        p.pose.position.z = 0.0;
        
        // 计算朝向：使用路径切线方向
        double yaw = 0.0;
        if (next_idx > path_idx) {
            double dx = plan.poses[next_idx].pose.position.x - plan.poses[path_idx].pose.position.x;
            double dy = plan.poses[next_idx].pose.position.y - plan.poses[path_idx].pose.position.y;
            if (std::hypot(dx, dy) > 0.001) {
                yaw = std::atan2(dy, dx);
            } else {
                // 使用路径点的朝向
                yaw = tf2::getYaw(plan.poses[path_idx].pose.orientation);
            }
        } else {
            // 最后一个点，使用目标朝向
            yaw = tf2::getYaw(plan.poses.back().pose.orientation);
        }
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        p.pose.orientation = tf2::toMsg(q);
        
        ref_traj.push_back(p);
    }
    
    return ref_traj;
}

void DcsShtMpcController::publishVisualizations(
    const std::vector<Polygon> & all_polygons,
    const std::vector<Polygon> & selected_polygons,
    const std::vector<geometry_msgs::msg::PoseStamped> & ref_traj,
    const std::vector<geometry_msgs::msg::PoseStamped> & mpc_traj)
{
    auto now = clock_->now();
    // 使用 costmap_ros 的 global_frame（通常是 odom）
    // 因为障碍物坐标是从 local costmap 中提取的
    std::string frame = costmap_ros_->getGlobalFrameID();

    // 1. All Obstacles (Gray)
    visualization_msgs::msg::MarkerArray all_mk;
    int id = 0;
    for(auto & poly : all_polygons) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = now;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.02; 
        m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.8;
        
        for(auto & pt : poly) {
            m.points.push_back(pt);
        }
        if(!poly.empty()) m.points.push_back(poly[0]); // close loop
        
        all_mk.markers.push_back(m);
    }
    all_obs_pub_->publish(all_mk);

    // 2. Selected (Red)
    visualization_msgs::msg::MarkerArray sel_mk;
    id = 0;
    for(auto & poly : selected_polygons) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = now;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05; // Thicker
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
        
        for(auto & pt : poly) {
            m.points.push_back(pt);
        }
        if(!poly.empty()) m.points.push_back(poly[0]); 
        
        sel_mk.markers.push_back(m);
    }
    sel_obs_pub_->publish(sel_mk);
    
    // 3. Publish Local Plan (Reference Trajectory)
    nav_msgs::msg::Path local_plan_msg;
    local_plan_msg.header.stamp = now;
    local_plan_msg.header.frame_id = frame;
    local_plan_msg.poses = ref_traj;
    local_plan_pub_->publish(local_plan_msg);
}

bool DcsShtMpcController::transformPlan(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & pose,
  nav_msgs::msg::Path & transformed_plan)
{
  if (plan.poses.empty()) {
    return false;
  }

  // Determine target frame (costmap frame)
  std::string costmap_frame = costmap_ros_->getGlobalFrameID();

  // If frames match, just copy
  if (plan.header.frame_id == costmap_frame) {
      transformed_plan = plan;
      return true;
  }

  // Look up transform
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      costmap_frame,
      plan.header.frame_id,
      tf2::TimePointZero); // Use latest transform
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Cannot transform plan: %s", ex.what());
    return false;
  }

  transformed_plan.header.frame_id = costmap_frame;
  transformed_plan.header.stamp = plan.header.stamp;

  // Transform poses
  // Simple heuristic: just transform all points. Nav2 ControllerServer prunes them anyway.
  for (const auto & p : plan.poses) {
    geometry_msgs::msg::PoseStamped t_pose;
    // Use manual transform because tf2::doTransform can be tricky with templates if headers missing
    // But geometry_msgs support is standard in tf2_geometry_msgs
    tf2::doTransform(p, t_pose, transform);
    transformed_plan.poses.push_back(t_pose);
  }
  
  return true;
}

bool DcsShtMpcController::checkCollision(
    const std::vector<geometry_msgs::msg::PoseStamped> & trajectory,
    const std::vector<Polygon> & obstacles)
{
    if (obstacles.empty() || trajectory.empty()) return false;
    
    // Conservative radius (circumscribed circle)
    double robot_radius = std::hypot(config_.car_length/2.0, config_.car_width/2.0); 
    
    // Pre-convert obstacles to OpenCV format for speed
    // 添加边界检查防止内存错误
    std::vector<std::vector<cv::Point2f>> cv_obs;
    cv_obs.reserve(obstacles.size());
    
    for(size_t i=0; i<obstacles.size(); ++i) {
        if (obstacles[i].size() < 3) continue; // 跳过无效多边形
        
        std::vector<cv::Point2f> poly;
        poly.reserve(obstacles[i].size());
        for(const auto& pt : obstacles[i]) {
            poly.push_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
        }
        cv_obs.push_back(std::move(poly));
    }
    
    if (cv_obs.empty()) return false;

    for(const auto& pose : trajectory) {
        cv::Point2f robot_pt(pose.pose.position.x, pose.pose.position.y);
        
        for(const auto& poly : cv_obs) {
            // pointPolygonTest: Positive=Inside, Negative=Outside, Zero=On Edge
            double dist = cv::pointPolygonTest(poly, robot_pt, true);
            
            // Collision if inside (dist>0) or within radius (dist > -radius)
            if (dist > -robot_radius) {
                RCLCPP_WARN(logger_, "Safety Shield: Predicted collision (dist=%.2f)", dist);
                return true;
            }
        }
    }
    return false;
}

}  // namespace dcs_nav_plugin

// Register Plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dcs_nav_plugin::DcsShtMpcController, nav2_core::Controller)
