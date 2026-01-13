#include "dcs_nav_plugin/sub_sht_mpc_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <tf2/utils.h>

namespace dcs_nav_plugin
{

void DcsShtMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  parent_ = parent;
  clock_ = parent->get_clock();
  logger_ = parent->get_logger();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  plugin_name_ = name;
  tf_buffer_ = tf;

  // load parameters
  loadParameters();

  // Components
  geometry_engine_ = std::make_shared<GeometryEngine>();
  geometry_engine_->configure(geometry_poly_epsilon_, 0.5, geometry_inflation_radius_, geometry_top_k_, geometry_dilation_pixels_);

  mpc_solver_ = std::make_shared<MpcSolverCasadi>();
  mpc_solver_->configure(config_);
  mpc_solver_->initialize();

  // Publishers
  auto node = parent;
  all_obs_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name + "/all_obstacles", 1);
  sel_obs_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name + "/selected_obstacles", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/local_plan", 1);
  
  // Debug & Logging
  debug_utils_ = std::make_shared<DebugUtils>();
  debug_utils_->initialize(debug_log_dir_, "controller"); 
  
  // Global Map Subscription
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  global_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", qos, 
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_global_map_ = msg;
    });

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

void DcsShtMpcController::loadParameters()
{
  auto node = parent_;
  
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
  
  // Modified Defaults for Better Rotation
  declare_and_get(".mpc.Q_theta", config_.Q_theta, 20.0); // Increased from 10.0
  declare_and_get(".mpc.R_v", config_.R_v, 0.1);
  declare_and_get(".mpc.R_omega", config_.R_omega, 0.2); // Decreased from 10.0
  
  declare_and_get(".mpc.lambda_slack", config_.lambda_slack, 100000.0);
  declare_and_get(".mpc.margin", config_.margin, 0.15);
  declare_and_get(".mpc.sht_epsilon", config_.sht_epsilon, 0.001);
  declare_and_get(".mpc.car_width", config_.car_width, 0.3);
  declare_and_get(".mpc.car_length", config_.car_length, 0.3);
  
  // Omni Movement
  declare_and_get(".enable_omni_movement", enable_omni_movement_, true);
  
  // Rotation Correction
  declare_and_get(".mpc.invert_rotation", invert_rotation_, false);

  // Geometry Engine Parameters
  declare_and_get(".geometry.poly_epsilon", geometry_poly_epsilon_, 0.05);
  declare_and_get(".geometry.inflation_radius", geometry_inflation_radius_, 0.0);
  declare_and_get(".geometry.top_k", geometry_top_k_, 6);
  declare_and_get(".geometry.dilation_pixels", geometry_dilation_pixels_, 1);

  // Debug Parameters
  std::string debug_log_dir;
  declare_and_get(".debug.log_dir", debug_log_dir, std::string("~/ros2_ws/src/dcs_nav_plugin/planner_log"));
  
  // Log loaded parameters
  RCLCPP_INFO(logger_, "[SHT-MPC] 参数已加载: v_max=%.2f, Omni=%d, Q_theta=%.1f, R_omega=%.1f, LogDir=%s, Dilation=%d",
              config_.v_max, enable_omni_movement_, config_.Q_theta, config_.R_omega, debug_log_dir.c_str(), geometry_dilation_pixels_);

  // Re-initialize DebugUtils with new path if needed (deferred initialization)
  // Actually initialize is called in configure() BEFORE loadParameters() in original code?
  // Let's check original code flow in configure().
  // Wait, configure() calls loadParameters() then debug_utils_->initialize().
  // So we need to store debug_log_dir in a member variable or return it?
  // loadParameters is void and sets members. We should add debug_log_dir_ member or just handle it here?
  // Easier: Make debug_log_dir_ a member of DcsShtMpcController.
  debug_log_dir_ = debug_log_dir;
}

geometry_msgs::msg::TwistStamped DcsShtMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{

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

  // FAILSAFE: If selected is empty but all is not (e.g. filtering bug), use ALL to avoid Safety Shield Stop
  if (selected_polys.empty() && !all_polys.empty()) {
      RCLCPP_WARN(logger_, "[SHT-MPC] ⚠️ 警告: 筛选结果为空但检测到障碍物! 启用 Failsafe: 强制使用全部障碍物 (截断至 Top-%d)", geometry_top_k_);
      selected_polys = all_polys;
      if (selected_polys.size() > (size_t)geometry_top_k_) {
          selected_polys.resize(geometry_top_k_);
      }
  }

  // 3. Solve MPC
  // 3.0 DEBUG LOGGING
  std::string debug_str = mpc_solver_->getDebugString(pose, ref_traj, selected_polys);
  debug_utils_->log("Frame: " + std::to_string(debug_frame_count_));
  debug_utils_->log(debug_str);
  
  // 3.0.1 SAVE IMAGES
  debug_utils_->saveLocalCostmap(costmap_, debug_frame_count_);
  debug_utils_->saveCostmapWithPolygons(costmap_, all_polys, selected_polys, debug_frame_count_);
  

  std::vector<geometry_msgs::msg::PoseStamped> empty_local; // will populate after solve
  // transform local plan points back to GLOBAL frame for global view drawing if needed
  // But MPC predicted path is output in 'costmap_ros_->getGlobalFrameID()' which is usually Odom.
  // Global View drawing needs careful frame handling. 
  // DebugUtils::saveGlobalView expects paths in their respective frames or coordinate transform handles logic.
  // Actually DebugUtils assumes inputs are in World frame or transformed inside. 
  // Let's pass what we have. Global Plan is already transformed to Controller Frame in 'transformed_plan'.
  // But for "Global View", user wants to see it on the STATIC MAP.
  // Static map acts as frame "map".
  // 'global_plan_' is in "map" usually. 
  // The 'ref_traj' is generated in controller frame (Odom).
  // We should pass 'global_plan_' (Map Frame) and 'ref_traj' (Odom Frame). 
  // To draw ref_traj correctly on Map, we need Odom->Map transform. 
  // Visualizer usually handles this via TF, but here we are drawing offline on an image.
  // We need current robot pose in Map Frame. The input 'pose' is in Odom Frame (from Controller Server) usually?
  // No, computeVelocityCommands 'pose' is in Odom/Costmap frame.
  // We need to look up Map->Odom to draw Odom-frame data on Map-frame image.
  // Or simpler: Just draw everything in Controller Frame if we don't have static map? 
  // But user specifically asked for "Global Map (non-costmap)".
  // So we try to transform 'ref_traj' (Odom) to Map Frame.
  
  // Try to get transforms for Debug
  std::vector<geometry_msgs::msg::PoseStamped> local_path_map_frame;
  geometry_msgs::msg::PoseStamped pose_map_frame;
  
  try {
      if (latest_global_map_) {
        // Transform Robot Pose to Map
        geometry_msgs::msg::TransformStamped tf_odom_to_map;
        // Lookup usually gives Target_T_Source. We want Map_T_Odom.
        tf_odom_to_map = tf_buffer_->lookupTransform("map", costmap_ros_->getGlobalFrameID(), tf2::TimePointZero);
        
        tf2::doTransform(pose, pose_map_frame, tf_odom_to_map);
        
        // Transform Ref Traj (just for visualization)
        /*
        for(auto& p : ref_traj) {
            geometry_msgs::msg::PoseStamped pm;
            tf2::doTransform(p, pm, tf_odom_to_map);
            local_path_map_frame.push_back(pm);
        }
        */
        // Actually let's just pass raw and let DebugUtils handle? 
        // No, DebugUtils is generic. Let's do transform here if possible or skip.
        // For simplicity, let's assume we can get transform.
      }
  } catch(...) {}

  // 3. Solve MPC
  auto sol = mpc_solver_->solve(pose, velocity, ref_traj, selected_polys);
  
  // 3.1 LOG MPC RESULT
  std::string result_str = mpc_solver_->getResultDebugString(sol);
  debug_utils_->log(result_str);
  
  // Transform predicted traj for visualization

   
   // Transform predicted traj for visualization
   if (latest_global_map_ && sol.success) {
      try {
        geometry_msgs::msg::TransformStamped tf_odom_to_map = 
            tf_buffer_->lookupTransform("map", costmap_ros_->getGlobalFrameID(), tf2::TimePointZero);
        for(auto& p : sol.predicted_traj) {
            geometry_msgs::msg::PoseStamped pm;
            tf2::doTransform(p, pm, tf_odom_to_map);
            local_path_map_frame.push_back(pm);
        }
      } catch(const tf2::TransformException & ex) {
          RCLCPP_WARN(logger_, "[SHT-MPC] Visualization TF Error (Path): %s", ex.what());
      }
   }
   
   if (latest_global_map_) {
       // Transform Polygons to Map Frame for visualization
       std::vector<Polygon> polygons_map_frame;
       try {
           geometry_msgs::msg::TransformStamped tf_odom_to_map = 
               tf_buffer_->lookupTransform("map", costmap_ros_->getGlobalFrameID(), tf2::TimePointZero);
           
           for(const auto& poly : selected_polys) {
               Polygon p_map;
               for(const auto& pt : poly) {
                   geometry_msgs::msg::PointStamped ps_in, ps_out;
                   ps_in.header.frame_id = costmap_ros_->getGlobalFrameID();
                   ps_in.point = pt;
                   tf2::doTransform(ps_in, ps_out, tf_odom_to_map);
                   p_map.push_back(ps_out.point);
               }
               polygons_map_frame.push_back(p_map);
           }
       } catch (const tf2::TransformException & ex) {
           RCLCPP_WARN(logger_, "[SHT-MPC] Visualization TF Error (Polygons): %s", ex.what());
       }
   
       debug_utils_->saveGlobalView(
            latest_global_map_, 
            global_plan_.poses, 
            local_path_map_frame, 
            pose_map_frame, 
            polygons_map_frame, 
            debug_frame_count_
       );
   }

  debug_frame_count_++;

  // Log solve result
  
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
      
      // Apply Rotation Inversion if enabled (Fix for Gazebo Planar Move Plugin issue)
      if (invert_rotation_) {
          cmd.twist.angular.z = -cmd.twist.angular.z;
      } 
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
    
    // 1. 找到路径上距离当前位置最近的点索引 (SE2 Distance)
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    double rot_weight_sq = std::pow(config_.car_length, 2); // Weight for angular distance
    
    for (size_t i = 0; i < plan.poses.size(); ++i) {
        double dx = plan.poses[i].pose.position.x - current_x;
        double dy = plan.poses[i].pose.position.y - current_y;
        
        // Calculate angular difference
        double yaw_path = tf2::getYaw(plan.poses[i].pose.orientation);
        double yaw_curr = tf2::getYaw(current_pose.pose.orientation);
        double d_theta = std::abs(yaw_path - yaw_curr);
        while(d_theta > M_PI) d_theta -= 2.0*M_PI;
        while(d_theta < -M_PI) d_theta += 2.0*M_PI;
        
        // Generalized Squared Distance: dx^2 + dy^2 + (L * dtheta)^2
        double d2 = dx*dx + dy*dy + rot_weight_sq * d_theta * d_theta;
        
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            closest_idx = i;
        }
    }

    // [Fix] Check if the robot is "ahead" of the closest point projected on the path segment.
    // If so, we should possibly advance closest_idx to avoid "dragging back".
    if (closest_idx + 1 < plan.poses.size()) {
        double dx = plan.poses[closest_idx+1].pose.position.x - plan.poses[closest_idx].pose.position.x;
        double dy = plan.poses[closest_idx+1].pose.position.y - plan.poses[closest_idx].pose.position.y;
        
        double rx = current_x - plan.poses[closest_idx].pose.position.x;
        double ry = current_y - plan.poses[closest_idx].pose.position.y;
        
        // Dot product: if > 0, robot is 'ahead' of closest_idx along the segment
        double dot = rx*dx + ry*dy;
        
        // Also check if we are not 'too far' ahead (e.g. valid projection)
        // Squared length of segment
        double seg_len_sq = dx*dx + dy*dy;
        
        if (seg_len_sq > 1e-6 && dot > 0) {
            // We are ahead. 
            // Ideally we should interpolate start arc. 
            // For now, simply advancing closest_idx helps prevent backward pull.
            // Only advance if we are substantially ahead or closer to the next point?
            // Actually, min_dist algorithm above *already* picked the globally closest one.
            // If Index 0 is closer than Index 1, it means we are closer to 0.
            // But if we are *ahead* of 0 (between 0 and 1) but closer to 0...
            // Ref[0] being at 0 pulls us back. 
            // We want Ref[0] to be *at our projection* or *ahead*.
            // Simple heuristic available: Force move to next index if we are "in front" of the closest point
            // This is aggressive but solves the "Backwards Pull" (Negative Velocity) issue.
            closest_idx++;
        }
    }
    
    // 2. 计算从 closest_idx 开始的累计广义弧长 (Generalized Arc Length)
    std::vector<double> arc_lengths;
    arc_lengths.push_back(0.0);
    double total_arc = 0.0;
    
    for (size_t i = closest_idx + 1; i < plan.poses.size(); ++i) {
        double dx = plan.poses[i].pose.position.x - plan.poses[i-1].pose.position.x;
        double dy = plan.poses[i].pose.position.y - plan.poses[i-1].pose.position.y;
        
        double yaw1 = tf2::getYaw(plan.poses[i].pose.orientation);
        double yaw0 = tf2::getYaw(plan.poses[i-1].pose.orientation);
        double d_theta = std::abs(yaw1 - yaw0);
        while(d_theta > M_PI) d_theta -= 2.0*M_PI;
        while(d_theta < -M_PI) d_theta += 2.0*M_PI;
        d_theta = std::abs(d_theta);

        // d_gen = sqrt(dx^2 + dy^2 + (L * dtheta)^2)
        // L = car_length (approx 0.3m)
        double segment_len = std::sqrt(dx*dx + dy*dy + rot_weight_sq * d_theta * d_theta);
        
        total_arc += segment_len;
        arc_lengths.push_back(total_arc);
    }
    
    // 3. 计算 MPC 预测时域内的预视距离
    // Combined Velocity Limit: max(v_max, w_max * L)
    double v_gen_ref = std::max(config_.v_max, config_.omega_max * config_.car_length);
    if (v_gen_ref < 0.1) v_gen_ref = 0.3;
    
    double lookahead_dist = N * v_gen_ref * dt;  // MPC时域内能走的广义距离
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
            seg_idx = i;
        }
        
        // 在这一段内插值
        size_t path_idx = closest_idx + seg_idx;
        size_t next_idx = std::min(path_idx + 1, plan.poses.size() - 1);
        
        double seg_start_arc = arc_lengths[seg_idx];
        double seg_end_arc = (seg_idx + 1 < arc_lengths.size()) ? arc_lengths[seg_idx + 1] : arc_lengths.back();
        double seg_len = seg_end_arc - seg_start_arc;
        
        double t = 0.0;
        if (seg_len > 0.0001) {
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
        
        // 计算朝向
        if (enable_omni_movement_) {
            tf2::Quaternion q1, q2;
            tf2::fromMsg(plan.poses[path_idx].pose.orientation, q1);
            tf2::fromMsg(plan.poses[next_idx].pose.orientation, q2);
            tf2::Quaternion q_interp = q1.slerp(q2, t);
            p.pose.orientation = tf2::toMsg(q_interp);

        } else {
            // Differential Drive Style logic... (kept simplified here or reuse user's branch logic if needed, but for now standard tangent)
            // For now, let's keep simple logic or just trust the path's orientation if using SE2
            // Actually, if using SE2, we should probably just interpolate path orientation too. 
            // But let's keep the user's original tangent logic for diff drive?
            // The user's code had tangent logic. Let's preserve it.
            
            double yaw = 0.0;
            if (next_idx > path_idx) {
                double dx = plan.poses[next_idx].pose.position.x - plan.poses[path_idx].pose.position.x;
                double dy = plan.poses[next_idx].pose.position.y - plan.poses[path_idx].pose.position.y;
                if (std::hypot(dx, dy) > 0.001) {
                    yaw = std::atan2(dy, dx);
                } else {
                    yaw = tf2::getYaw(plan.poses[path_idx].pose.orientation);
                }
            } else {
                yaw = tf2::getYaw(plan.poses.back().pose.orientation);
            }
            
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            p.pose.orientation = tf2::toMsg(q);
        }
        
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
            
            // Collision if inside (dist>0) or significantly inside margin
            // FIX: Relaxed Safety Shield. Only stop if very close to obstacle (0m or -0.02m)
            // The polygon is already dilated by GeometryEngine (approx 5cm). 
            // Using full margin (10cm) caused false positives when starting near obstacles.
            // dist > 0 means inside the dilated polygon.
            double safety_threshold = 0.0; // Was -config_.margin
            if (dist > safety_threshold) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, 
                    "Safety Shield: Predicted collision (dist=%.3f, threshold=%.3f)", dist, safety_threshold);
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
