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

    // 1. Find Closest Point Index
    // (Optimization: plan is transformed to costmap frame now, so comparison is valid)
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for(size_t i=0; i<plan.poses.size(); ++i) {
        double dx = plan.poses[i].pose.position.x - current_pose.pose.position.x;
        double dy = plan.poses[i].pose.position.y - current_pose.pose.position.y;
        double d2 = dx*dx + dy*dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            closest_idx = i;
        }
    }
    
    // 2. Generate Time-Based Reference Points
    // Assume constant target velocity for reference generation
    double v_ref = config_.v_max; 
    if (v_ref < 0.1) v_ref = 0.5; // Minimal speed

    for(int k=0; k<=N; ++k) {
        double target_dist = k * v_ref * dt;
        
        // Walk forward from closest_idx to find the segment containing target_dist
        double accumulated_dist = 0.0;
        size_t search_idx = closest_idx;
        
        // Find segment [p1, p2]
        while(search_idx + 1 < plan.poses.size()) {
            double ds = std::hypot(
                plan.poses[search_idx+1].pose.position.x - plan.poses[search_idx].pose.position.x,
                plan.poses[search_idx+1].pose.position.y - plan.poses[search_idx].pose.position.y
            );
            if (accumulated_dist + ds >= target_dist) {
                // Interpolate in this segment
                double ratio = (target_dist - accumulated_dist) / std::max(ds, 1e-6);
                
                geometry_msgs::msg::PoseStamped p;
                p.header = plan.poses[search_idx].header;
                
                // Linear Interp Position (Approximates Spline on dense path)
                p.pose.position.x = plan.poses[search_idx].pose.position.x + ratio * (plan.poses[search_idx+1].pose.position.x - plan.poses[search_idx].pose.position.x);
                p.pose.position.y = plan.poses[search_idx].pose.position.y + ratio * (plan.poses[search_idx+1].pose.position.y - plan.poses[search_idx].pose.position.y);
                
                // Slerp Orientation - 使用路径切线方向而非全局规划朝向
                // 计算当前段的切线方向作为参考朝向（更准确）
                double dx_seg = plan.poses[search_idx+1].pose.position.x - plan.poses[search_idx].pose.position.x;
                double dy_seg = plan.poses[search_idx+1].pose.position.y - plan.poses[search_idx].pose.position.y;
                double yaw = std::atan2(dy_seg, dx_seg);  // 路径切线方向
                
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                p.pose.orientation = tf2::toMsg(q);
                
                ref_traj.push_back(p);
                break;
            }
            accumulated_dist += ds;
            search_idx++;
        }
        
        // If end of path reached - 使用最后一段的切线方向而非终点朝向
        if (ref_traj.size() <= (size_t)k) {
            geometry_msgs::msg::PoseStamped p = plan.poses.back();
            
            // 计算最后一段的切线方向以保持朝向连续性
            if (plan.poses.size() >= 2) {
                size_t n = plan.poses.size();
                double dx_last = plan.poses[n-1].pose.position.x - plan.poses[n-2].pose.position.x;
                double dy_last = plan.poses[n-1].pose.position.y - plan.poses[n-2].pose.position.y;
                double yaw_last = std::atan2(dy_last, dx_last);
                
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw_last);
                p.pose.orientation = tf2::toMsg(q);
            }
            
            ref_traj.push_back(p);
        }
    }
    
    // 检测是否所有参考点都在终点（接近目标时的特殊情况）
    if (ref_traj.size() >= 2) {
        const auto& first = ref_traj.front();
        const auto& last = ref_traj.back();
        double dist = std::hypot(
            last.pose.position.x - first.pose.position.x,
            last.pose.position.y - first.pose.position.y);
        
        // 如果所有参考点位置几乎相同（都在终点），创建朝向平滑过渡
        if (dist < 0.1) {  // 阈值 0.1m
            double current_yaw = tf2::getYaw(current_pose.pose.orientation);
            double goal_yaw = tf2::getYaw(last.pose.orientation);
            
            // 归一化目标朝向到当前朝向附近
            while (goal_yaw - current_yaw > M_PI) goal_yaw -= 2.0 * M_PI;
            while (goal_yaw - current_yaw < -M_PI) goal_yaw += 2.0 * M_PI;
            
            // 在预测时域内平滑过渡朝向
            size_t N = ref_traj.size();
            for (size_t i = 0; i < N; ++i) {
                double t = static_cast<double>(i) / static_cast<double>(N - 1);  // 0 到 1
                double interp_yaw = current_yaw + t * (goal_yaw - current_yaw);
                
                tf2::Quaternion q;
                q.setRPY(0, 0, interp_yaw);
                ref_traj[i].pose.orientation = tf2::toMsg(q);
            }
            
            return ref_traj;
        }
    }
    
    // 强制朝向平滑 - 限制相邻点之间的朝向变化率
    // 这是解决高松弛值和抽动的关键修复
    if (!ref_traj.empty()) {
        double prev_yaw = tf2::getYaw(current_pose.pose.orientation);  // 从当前朝向开始
        
        // 每个参考点允许的最大朝向变化（根据预测时间步长和最大角速度）
        // dt = 0.1s (10Hz), omega_max = 0.5 rad/s -> max_delta = 0.05 rad per step
        // 但为了更平滑，我们使用稍大的值
        const double max_yaw_rate_per_step = 0.1;  // rad per reference point
        
        for (auto & p : ref_traj) {
            double target_yaw = tf2::getYaw(p.pose.orientation);
            
            // 归一化目标朝向到与 prev_yaw 最接近的等价角度
            while (target_yaw - prev_yaw > M_PI) target_yaw -= 2.0 * M_PI;
            while (target_yaw - prev_yaw < -M_PI) target_yaw += 2.0 * M_PI;
            
            // 限制朝向变化率
            double delta = target_yaw - prev_yaw;
            if (delta > max_yaw_rate_per_step) delta = max_yaw_rate_per_step;
            if (delta < -max_yaw_rate_per_step) delta = -max_yaw_rate_per_step;
            
            double smoothed_yaw = prev_yaw + delta;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, smoothed_yaw);
            p.pose.orientation = tf2::toMsg(q);
            
            prev_yaw = smoothed_yaw;  // 更新参考（使用平滑后的值）
        }
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
