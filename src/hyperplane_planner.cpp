#include "dcs_nav_plugin/hyperplane_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

// 注册插件
PLUGINLIB_EXPORT_CLASS(dcs_nav_plugin::HyperplanePlanner, nav2_core::GlobalPlanner)

namespace dcs_nav_plugin
{

HyperplanePlanner::HyperplanePlanner()
  : costmap_(nullptr)
{
}

HyperplanePlanner::~HyperplanePlanner()
{
}

void HyperplanePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

#ifndef DCS_WITH_CASADI
  RCLCPP_ERROR(node->get_logger(), 
    "HyperplanePlanner requires CasADi but it was not found during compilation!");
  throw std::runtime_error("CasADi not available");
#endif

  RCLCPP_INFO(node->get_logger(), "Configuring HyperplanePlanner: %s", name_.c_str());

  // === 车辆几何参数 ===
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".vehicle_length", rclcpp::ParameterValue(0.20));
  node->get_parameter(name + ".vehicle_length", vehicle_length_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".vehicle_width", rclcpp::ParameterValue(0.17));
  node->get_parameter(name + ".vehicle_width", vehicle_width_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".vehicle_wheelbase", rclcpp::ParameterValue(0.16));
  node->get_parameter(name + ".vehicle_wheelbase", vehicle_wheelbase_);

  // === 优化设置 ===
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".N", rclcpp::ParameterValue(20));
  node->get_parameter(name + ".N", N_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".M", rclcpp::ParameterValue(5));
  node->get_parameter(name + ".M", M_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_vel_xy", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".max_vel_xy", max_vel_xy_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".max_vel_theta", max_vel_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_acc_xy", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".max_acc_xy", max_acc_xy_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_acc_theta", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".max_acc_theta", max_acc_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tf_max", rclcpp::ParameterValue(60.0));
  node->get_parameter(name + ".tf_max", tf_max_);

  // === 代价权重 ===
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".weight_smooth", rclcpp::ParameterValue(100.0));
  node->get_parameter(name + ".weight_smooth", weight_smooth_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".weight_time", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".weight_time", weight_time_);

  // === 求解器参数 ===
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".solver_max_iter", rclcpp::ParameterValue(3000.0));
  node->get_parameter(name + ".solver_max_iter", solver_max_iter_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".solver_tol", rclcpp::ParameterValue(1e-4));
  node->get_parameter(name + ".solver_tol", solver_tol_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".solver_time_limit", rclcpp::ParameterValue(10.0));
  node->get_parameter(name + ".solver_time_limit", solver_time_limit_);

  // === 障碍物处理 ===
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_inflation", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".obstacle_inflation", obstacle_inflation_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lambda_max", rclcpp::ParameterValue(1e5));
  node->get_parameter(name + ".lambda_max", lambda_max_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".mu_max", rclcpp::ParameterValue(1e5));
  node->get_parameter(name + ".mu_max", mu_max_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".eps", rclcpp::ParameterValue(1e-4));
  node->get_parameter(name + ".eps", eps_);

  // 设置画布边界（从代价地图获取）
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double size_x = costmap_->getSizeInMetersX();
  double size_y = costmap_->getSizeInMetersY();

  // 画布约束: [上, 右, 下, 左]
  A_canvas_ = casadi::DM(std::vector<std::vector<double>>{
    {0, 1}, {1, 0}, {0, -1}, {-1, 0}
  });
  b_canvas_ = casadi::DM(std::vector<double>{
    origin_y + size_y, origin_x + size_x, -origin_y, -origin_x
  });

  RCLCPP_INFO(node->get_logger(), 
    "HyperplanePlanner configured: N=%d, M=%d, vehicle: %.2fm x %.2fm",
    N_, M_, vehicle_length_, vehicle_width_);

  // 初始化 HybridAStar 用于 warm start
  hybrid_astar_ = std::make_unique<HybridAStar>();
  hybrid_astar_->configure(parent, name + "_hybrid_astar", tf, costmap_ros);
  hybrid_astar_->activate();
  RCLCPP_INFO(node->get_logger(), "HybridAStar warm start initialized");
}

void HyperplanePlanner::cleanup()
{
  RCLCPP_INFO(rclcpp::get_logger("HyperplanePlanner"), 
    "Cleaning up plugin %s", name_.c_str());
}

void HyperplanePlanner::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("HyperplanePlanner"), 
    "Activating plugin %s", name_.c_str());
}

void HyperplanePlanner::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("HyperplanePlanner"), 
    "Deactivating plugin %s", name_.c_str());
}

nav_msgs::msg::Path HyperplanePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = start.header.stamp;
  global_path.header.frame_id = global_frame_;

  auto node = node_.lock();
  if (!node || !costmap_) {
    RCLCPP_ERROR(rclcpp::get_logger("HyperplanePlanner"), 
      "Node or costmap not available!");
    return global_path;
  }

  RCLCPP_INFO(node->get_logger(), "HyperplanePlanner::createPlan called");

  // Check if start or goal is in lethal obstacle
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    unsigned char cost = costmap_->getCost(mx_start, my_start);
    RCLCPP_INFO(node->get_logger(), "Start pose cost: %d", (int)cost);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(node->get_logger(), "Start pose is in lethal obstacle/inflation!");
    }
  }
  
  if (costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    unsigned char cost = costmap_->getCost(mx_goal, my_goal);
    RCLCPP_INFO(node->get_logger(), "Goal pose cost: %d", (int)cost);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(node->get_logger(), "Goal pose is in lethal obstacle/inflation!");
    }
  }

#ifdef DCS_WITH_CASADI
  RCLCPP_INFO(node->get_logger(), 
    "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // 1. 提取起点和终点状态 - 全向轮6维状态 [x, y, theta, vx, vy, omega]
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_theta = tf2::getYaw(start.pose.orientation);

  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double goal_theta = tf2::getYaw(goal.pose.orientation);

  // 全向轮状态向量: [x, y, theta, vx, vy, omega]
  std::vector<double> start_state = {start_x, start_y, start_theta, 0.0, 0.0, 0.0};
  std::vector<double> goal_state = {goal_x, goal_y, goal_theta, 0.0, 0.0, 0.0};

  // 2. 首先生成 Warm Start 初始轨迹（用于过滤障碍物）
  casadi::DM X_init, U_init;
  nav_msgs::msg::Path warm_path;
  if (hybrid_astar_) {
    warm_path = hybrid_astar_->createPlan(start, goal);
    RCLCPP_INFO(node->get_logger(), "HybridAStar generated %zu poses for path corridor", warm_path.poses.size());
  }
  
  if (!generateWarmStart(start, goal, X_init, U_init)) {
    RCLCPP_WARN(node->get_logger(), "Warm start failed, using linear interpolation");
  }

  // 3. 提取所有障碍物
  double map_origin_x = costmap_->getOriginX();
  double map_origin_y = costmap_->getOriginY();
  double map_size_x = costmap_->getSizeInMetersX();
  double map_size_y = costmap_->getSizeInMetersY();

  std::vector<Obstacle> all_obstacles = extractObstacles(
    map_origin_x, map_origin_x + map_size_x,
    map_origin_y, map_origin_y + map_size_y);
  RCLCPP_INFO(node->get_logger(), "Extracted %zu obstacles from entire costmap", all_obstacles.size());

  // 4. 根据路径走廊过滤障碍物
  const double corridor_width = 1.5;  // 走廊宽度（米）
  std::vector<Obstacle> obstacles;
  if (!warm_path.poses.empty()) {
    obstacles = filterObstaclesByPath(all_obstacles, warm_path, corridor_width);
    RCLCPP_INFO(node->get_logger(), "Filtered to %zu obstacles within %.1fm corridor", 
                obstacles.size(), corridor_width);
  } else {
    // 如果没有路径，使用所有障碍物
    obstacles = all_obstacles;
    RCLCPP_WARN(node->get_logger(), "No path for filtering, using all %zu obstacles", obstacles.size());
  }

  // 5. 求解优化问题
  casadi::DM X_opt, U_opt;
  double tf_opt;
  
  auto start_time = std::chrono::steady_clock::now();
  bool success = solveOptimization(start_state, goal_state, obstacles, 
                                   X_init, U_init, X_opt, U_opt, tf_opt);
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  if (success) {
    RCLCPP_INFO(node->get_logger(), 
      "Optimization succeeded! Time Horizon Tf: %.2f s, Computation Time: %ld ms", 
      tf_opt, duration.count());
    global_path = trajectoryToPath(X_opt, tf_opt, start);
    RCLCPP_INFO(node->get_logger(), "Generated path with %zu poses", global_path.poses.size());
  } else {
    RCLCPP_WARN(node->get_logger(), "Optimization failed after %ld ms", duration.count());
  }
#else
  RCLCPP_ERROR(node->get_logger(), "CasADi not available!");
#endif

  return global_path;
}

#ifdef DCS_WITH_CASADI

std::vector<Obstacle> HyperplanePlanner::extractObstacles(double min_x, double max_x, double min_y, double max_y)
{
  std::vector<Obstacle> obstacles;
  
  // === 凸包递归分割算法 (Convex Hull with Recursive Split) ===
  // 1. 获取所有连通域
  // 2. 对每个连通域计算凸包，检查填充率
  // 3. 若填充率 < 阈值，沿主轴分割像素，递归处理
  // 4. 凸包保证100%覆盖所有像素点
  
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  double resolution = costmap_->getResolution();
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  
  // 参数 - 调整以减少障碍物数量，提高性能
  const double fill_ratio_threshold = 0.50;  // 填充率阈值（提高以减少分割）
  const int min_pixel_count = 1;             // 最小像素数（确保覆盖所有像素）
  const int max_recursion_depth = 6;         // 最大递归深度（降低以减少障碍物）
  
  // 1. 从costmap创建二值图像
  cv::Mat binary_map(size_y, size_x, CV_8UC1, cv::Scalar(0));
  
  for (unsigned int j = 0; j < size_y; ++j) {
    for (unsigned int i = 0; i < size_x; ++i) {
      if (costmap_->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        binary_map.at<uchar>(size_y - 1 - j, i) = 255;
      }
    }
  }
  
  // 2. 找所有连通域
  cv::Mat labels;
  int num_labels = cv::connectedComponents(binary_map, labels, 8);
  
  RCLCPP_INFO(rclcpp::get_logger("HyperplanePlanner"), 
    "Found %d connected components, applying convex hull recursive split...", num_labels - 1);
  
  // 3. 凸包递归分割函数
  std::function<void(const cv::Mat&, int)> recursiveSplitMask;
  
  recursiveSplitMask = [&](const cv::Mat& mask, int depth) {
    // 获取所有障碍物像素坐标
    std::vector<cv::Point> pixels;
    for (int y = 0; y < mask.rows; ++y) {
      for (int x = 0; x < mask.cols; ++x) {
        if (mask.at<uchar>(y, x) > 0) {
          pixels.emplace_back(x, y);
        }
      }
    }
    
    if (static_cast<int>(pixels.size()) < min_pixel_count) return;
    
    // 计算凸包
    std::vector<cv::Point> hull;
    cv::convexHull(pixels, hull);
    
    if (hull.size() < 3) {
      // 不足3个点，无法形成凸包，创建一个小矩形
      if (pixels.size() >= 1) {
        // 用单个像素创建一个小正方形
        cv::Rect bbox = cv::boundingRect(pixels);
        hull.clear();
        hull.push_back(cv::Point(bbox.x, bbox.y));
        hull.push_back(cv::Point(bbox.x + bbox.width, bbox.y));
        hull.push_back(cv::Point(bbox.x + bbox.width, bbox.y + bbox.height));
        hull.push_back(cv::Point(bbox.x, bbox.y + bbox.height));
      } else {
        return;
      }
    }
    
    // 计算凸包面积
    double hull_area = cv::contourArea(hull);
    if (hull_area < 1.0) hull_area = 1.0;
    
    // 计算填充率 = 像素数 / 凸包面积
    double pixel_count = static_cast<double>(pixels.size());
    double fill_ratio = pixel_count / hull_area;
    
    // 计算OBB用于分割方向
    cv::RotatedRect obb = cv::minAreaRect(pixels);
    
    // 停止条件
    bool should_stop = (fill_ratio >= fill_ratio_threshold) || 
                       (depth >= max_recursion_depth) ||
                       (std::max(obb.size.width, obb.size.height) < 4) ||
                       (pixels.size() <= 3);
    
    if (should_stop) {
      // 转换凸包顶点为世界坐标
      std::vector<std::pair<double, double>> world_vertices;
      double cx = 0, cy = 0;
      
      for (const auto& pt : hull) {
        // 添加0.5像素偏移，使坐标对应像素中心而非角落
        double wx = origin_x + (pt.x + 0.5) * resolution;
        double wy = origin_y + (size_y - 1 - pt.y + 0.5) * resolution;
        world_vertices.push_back({wx, wy});
        cx += wx;
        cy += wy;
      }
      cx /= hull.size();
      cy /= hull.size();
      
      // 膨胀
      for (auto& v : world_vertices) {
        double dx = v.first - cx;
        double dy = v.second - cy;
        double len = std::sqrt(dx*dx + dy*dy);
        if (len > 1e-6) {
          v.first += (dx / len) * obstacle_inflation_;
          v.second += (dy / len) * obstacle_inflation_;
        }
      }
      
      // 过滤超出ROI
      bool in_roi = false;
      for (const auto& v : world_vertices) {
        if (v.first >= min_x && v.first <= max_x && 
            v.second >= min_y && v.second <= max_y) {
          in_roi = true;
          break;
        }
      }
      if (!in_roi) return;
      
      // 构建N个半平面约束（凸包有N条边）
      int n_edges = static_cast<int>(world_vertices.size());
      casadi::DM A(n_edges, 2);
      casadi::DM b(n_edges, 1);
      
      for (int e = 0; e < n_edges; ++e) {
        int next_e = (e + 1) % n_edges;
        double x1 = world_vertices[e].first;
        double y1 = world_vertices[e].second;
        double x2 = world_vertices[next_e].first;
        double y2 = world_vertices[next_e].second;
        
        double edge_dx = x2 - x1;
        double edge_dy = y2 - y1;
        double edge_len = std::sqrt(edge_dx*edge_dx + edge_dy*edge_dy);
        
        if (edge_len < 1e-6) { edge_dx = 1.0; edge_dy = 0.0; edge_len = 1.0; }
        
        // 外法线（逆时针顺序）
        double nx = edge_dy / edge_len;
        double ny = -edge_dx / edge_len;
        double offset = nx * x1 + ny * y1;
        
        A(e, 0) = nx;
        A(e, 1) = ny;
        b(e, 0) = offset;
      }
      
      obstacles.emplace_back(A, b);
      return;
    }
    
    // 需要分割：沿OBB长轴的垂直方向分割
    double angle_rad = obb.angle * CV_PI / 180.0;
    
    double split_nx, split_ny;
    if (obb.size.width >= obb.size.height) {
      split_nx = std::cos(angle_rad);
      split_ny = std::sin(angle_rad);
    } else {
      split_nx = -std::sin(angle_rad);
      split_ny = std::cos(angle_rad);
    }
    
    // 计算质心
    double centroid_x = 0, centroid_y = 0;
    for (const auto& pt : pixels) {
      centroid_x += pt.x;
      centroid_y += pt.y;
    }
    centroid_x /= pixels.size();
    centroid_y /= pixels.size();
    
    // 创建两个子掩码
    cv::Mat mask1 = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::Mat mask2 = cv::Mat::zeros(mask.size(), CV_8UC1);
    
    for (const auto& pt : pixels) {
      double dot = (pt.x - centroid_x) * split_nx + (pt.y - centroid_y) * split_ny;
      if (dot >= 0) {
        mask1.at<uchar>(pt.y, pt.x) = 255;
      } else {
        mask2.at<uchar>(pt.y, pt.x) = 255;
      }
    }
    
    // 递归处理
    if (cv::countNonZero(mask1) >= min_pixel_count) {
      recursiveSplitMask(mask1, depth + 1);
    }
    if (cv::countNonZero(mask2) >= min_pixel_count) {
      recursiveSplitMask(mask2, depth + 1);
    }
  };
  
  // 4. 对每个连通域应用递归分割
  for (int label_id = 1; label_id < num_labels; ++label_id) {
    cv::Mat component_mask = (labels == label_id);
    component_mask.convertTo(component_mask, CV_8UC1, 255);
    
    if (cv::countNonZero(component_mask) >= min_pixel_count) {
      recursiveSplitMask(component_mask, 0);
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("HyperplanePlanner"), 
    "Extracted %zu Convex Hull obstacles using Recursive Split (fill_ratio>=%.0f%%)", 
    obstacles.size(), fill_ratio_threshold * 100);
  
  // 导出障碍物数据
  static bool first_call = true;
  if (first_call) {
    std::ofstream debug_file("/home/dcs/ros2_ws/obstacles_debug.txt");
    debug_file << "# Convex Hull: ID cx cy num_edges\n";
    for (size_t i = 0; i < obstacles.size(); ++i) {
      const auto& obs = obstacles[i];
      int n_edges = obs.A.size1();
      
      // 计算凸包中心（从半平面交点）
      double cx = 0, cy = 0;
      int vertex_count = 0;
      for (int e = 0; e < n_edges; ++e) {
        int next_e = (e + 1) % n_edges;
        double a1 = obs.A(e, 0).scalar();
        double b1 = obs.A(e, 1).scalar();
        double c1 = obs.b(e).scalar();
        double a2 = obs.A(next_e, 0).scalar();
        double b2 = obs.A(next_e, 1).scalar();
        double c2 = obs.b(next_e).scalar();
        double det = a1 * b2 - a2 * b1;
        if (std::abs(det) > 1e-9) {
          cx += (c1 * b2 - c2 * b1) / det;
          cy += (a1 * c2 - a2 * c1) / det;
          vertex_count++;
        }
      }
      if (vertex_count > 0) {
        cx /= vertex_count;
        cy /= vertex_count;
      }
      debug_file << i << " " << cx << " " << cy << " " << n_edges << "\n";
    }
    debug_file.close();
    RCLCPP_INFO(rclcpp::get_logger("HyperplanePlanner"), 
      "Convex Hull数据已导出到 /home/dcs/ros2_ws/obstacles_debug.txt");
    first_call = false;
  }
  
  return obstacles;
}

std::vector<Obstacle> HyperplanePlanner::filterObstaclesByPath(
  const std::vector<Obstacle>& obstacles,
  const nav_msgs::msg::Path& path,
  double corridor_width)
{
  if (path.poses.empty()) {
    return obstacles;  // 没有路径，返回所有障碍物
  }

  std::vector<Obstacle> filtered;
  filtered.reserve(obstacles.size() / 2);  // 预估一半左右

  for (const auto& obs : obstacles) {
    // 计算障碍物中心（从半平面交点）
    int n_edges = obs.A.size1();
    double obs_cx = 0, obs_cy = 0;
    int vertex_count = 0;
    
    for (int i = 0; i < n_edges; ++i) {
      int j = (i + 1) % n_edges;
      double a1 = obs.A(i, 0).scalar();
      double b1 = obs.A(i, 1).scalar();
      double c1 = obs.b(i).scalar();
      double a2 = obs.A(j, 0).scalar();
      double b2 = obs.A(j, 1).scalar();
      double c2 = obs.b(j).scalar();
      double det = a1 * b2 - a2 * b1;
      if (std::abs(det) > 1e-9) {
        obs_cx += (c1 * b2 - c2 * b1) / det;
        obs_cy += (a1 * c2 - a2 * c1) / det;
        vertex_count++;
      }
    }
    
    if (vertex_count == 0) continue;
    obs_cx /= vertex_count;
    obs_cy /= vertex_count;

    // 检查障碍物是否在任意路径点的走廊范围内
    bool in_corridor = false;
    for (const auto& pose : path.poses) {
      double px = pose.pose.position.x;
      double py = pose.pose.position.y;
      double dist = std::sqrt((obs_cx - px)*(obs_cx - px) + (obs_cy - py)*(obs_cy - py));
      
      if (dist <= corridor_width) {
        in_corridor = true;
        break;
      }
    }

    if (in_corridor) {
      filtered.push_back(obs);
    }
  }

  return filtered;
}

casadi::DM HyperplanePlanner::getVehicleVertices(double x, double y, double theta)
{
  // 车辆局部坐标系中的四个角点
  double half_length = vehicle_length_ / 2.0;
  double half_width = vehicle_width_ / 2.0;

  // [前左, 前右, 后右, 后左]
  casadi::DM V_local = casadi::DM(std::vector<std::vector<double>>{
    { half_length,  half_length, -half_length, -half_length},
    { half_width,  -half_width,  -half_width,   half_width}
  });

  // 旋转矩阵
  casadi::DM R = casadi::DM(std::vector<std::vector<double>>{
    {std::cos(theta), -std::sin(theta)},
    {std::sin(theta),  std::cos(theta)}
  });

  // 平移向量
  casadi::DM T = casadi::DM(std::vector<double>{x, y});

  // V_world = R * V_local + T
  casadi::DM V_world = casadi::DM::mtimes(R, V_local);
  for (int i = 0; i < 4; ++i) {
    V_world(0, i) = V_world(0, i).scalar() + T(0).scalar();
    V_world(1, i) = V_world(1, i).scalar() + T(1).scalar();
  }

  return V_world;
}

bool HyperplanePlanner::generateWarmStart(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  casadi::DM& X_init,
  casadi::DM& U_init)
{
  auto node = node_.lock();
  if (!node || !hybrid_astar_) {
    return false;
  }

  // 使用 HybridAStar 生成初始路径
  nav_msgs::msg::Path astar_path = hybrid_astar_->createPlan(start, goal);
  
  if (astar_path.poses.empty()) {
    RCLCPP_WARN(node->get_logger(), "HybridAStar returned empty path for warm start");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "HybridAStar generated %zu poses for warm start", 
              astar_path.poses.size());

  // 将 HybridAStar 路径插值到 N+1 个点
  // 全向轮状态: [x, y, theta, vx, vy, omega] = 6维
  X_init = casadi::DM::zeros(6, N_ + 1);
  U_init = casadi::DM::zeros(3, N_);

  size_t path_size = astar_path.poses.size();
  
  for (int k = 0; k <= N_; ++k) {
    double alpha = static_cast<double>(k) / N_;
    size_t idx = static_cast<size_t>(alpha * (path_size - 1));
    idx = std::min(idx, path_size - 1);
    
    const auto& pose = astar_path.poses[idx].pose;
    X_init(0, k) = pose.position.x;
    X_init(1, k) = pose.position.y;
    X_init(2, k) = tf2::getYaw(pose.orientation);
    // vx, vy, omega 初始化为0
    X_init(3, k) = 0.0;
    X_init(4, k) = 0.0;
    X_init(5, k) = 0.0;
  }

  // 根据相邻位置估计速度 (简化版)
  double estimated_tf = 10.0;  // 估计总时间
  double dt = estimated_tf / N_;
  
  for (int k = 0; k < N_; ++k) {
    double dx = X_init(0, k+1).scalar() - X_init(0, k).scalar();
    double dy = X_init(1, k+1).scalar() - X_init(1, k).scalar();
    
    // 估计速度
    double vx_est = dx / dt;
    double vy_est = dy / dt;
    
    // 限制速度范围
    vx_est = std::clamp(vx_est, -max_vel_xy_, max_vel_xy_);
    vy_est = std::clamp(vy_est, -max_vel_xy_, max_vel_xy_);
    
    X_init(3, k) = vx_est;
    X_init(4, k) = vy_est;
    
    // 控制量初始化为0
    U_init(0, k) = 0.0;
    U_init(1, k) = 0.0;
    U_init(2, k) = 0.0;
  }
  
  return true;
}

bool HyperplanePlanner::solveOptimization(
  const std::vector<double>& start_state,
  const std::vector<double>& goal_state,
  const std::vector<Obstacle>& obstacles,
  const casadi::DM& X_init,
  const casadi::DM& U_init,
  casadi::DM& X_out,
  casadi::DM& U_out,
  double& tf_out)
{
  using namespace casadi;

  auto node = node_.lock();
  if (!node) return false;

  try {
    // === 1. 创建优化问题 ===
    Opti opti = Opti();

    // === 2. 定义决策变量（全向轮6维状态）===
    // 状态: X = [x, y, theta, vx, vy, omega]^T (6 x N+1)
    MX X = opti.variable(6, N_ + 1);
    // 控制: U = [ax, ay, a_omega]^T (3 x N)
    MX U = opti.variable(3, N_);
    // 总时间: Tf (标量)
    MX Tf = opti.variable();

    // === 3. 定义符号变量（用于动力学）===
    MX x_sym = MX::sym("x");
    MX y_sym = MX::sym("y");
    MX theta_sym = MX::sym("theta");
    MX vx_sym = MX::sym("vx");
    MX vy_sym = MX::sym("vy");
    MX omega_sym = MX::sym("omega");
    MX state = MX::vertcat({x_sym, y_sym, theta_sym, vx_sym, vy_sym, omega_sym});

    MX ax_sym = MX::sym("ax");
    MX ay_sym = MX::sym("ay");
    MX a_omega_sym = MX::sym("a_omega");
    MX control = MX::vertcat({ax_sym, ay_sym, a_omega_sym});

    // === 4. 全向轮动力学模型 ===
    // dx/dt = vx
    // dy/dt = vy
    // dtheta/dt = omega
    // dvx/dt = ax
    // dvy/dt = ay
    // domega/dt = a_omega
    MX x_dot = MX::vertcat({
      vx_sym,
      vy_sym,
      omega_sym,
      ax_sym,
      ay_sym,
      a_omega_sym
    });

    // 代价函数（阶段性代价）- 控制平滑性
    MX stage_cost = weight_smooth_ * (ax_sym * ax_sym + ay_sym * ay_sym + a_omega_sym * a_omega_sym);

    Function dynamics("f", {state, control}, {x_dot, stage_cost});

    // === 5. RK4 积分器 ===
    double h = 1.0 / N_;  // 归一化时间步长
    double dt = h / M_;   // 子步长

    MX X_k = MX::sym("X_k", 6);
    MX U_k = MX::sym("U_k", 3);
    MX Tf_sym = MX::sym("Tf");
    
    MX X_next = X_k;
    MX Q = 0;

    for (int m = 0; m < M_; ++m) {
      auto k1 = dynamics(std::vector<MX>{X_next, U_k});
      auto k2 = dynamics(std::vector<MX>{X_next + dt/2 * Tf_sym * k1[0], U_k});
      auto k3 = dynamics(std::vector<MX>{X_next + dt/2 * Tf_sym * k2[0], U_k});
      auto k4 = dynamics(std::vector<MX>{X_next + dt * Tf_sym * k3[0], U_k});
      
      X_next = X_next + dt/6 * Tf_sym * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
      Q = Q + dt/6 * Tf_sym * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
    }

    Function integrator("F", {X_k, U_k, Tf_sym}, {X_next, Q});

    // === 6. 添加约束 ===
    
    // 初始状态约束
    opti.subject_to(X(Slice(), 0) == DM(start_state));
    
    // 终点状态约束 - 只约束位置和朝向
    opti.subject_to(X(0, N_) == goal_state[0]);  // x
    opti.subject_to(X(1, N_) == goal_state[1]);  // y
    opti.subject_to(X(2, N_) == goal_state[2]);  // theta

    // 时间约束
    opti.subject_to(Tf >= eps_);
    opti.subject_to(Tf <= tf_max_);

    // 动力学约束
    MX J = 0;  // 总代价
    for (int k = 0; k < N_; ++k) {
      auto result = integrator(std::vector<MX>{X(Slice(), k), U(Slice(), k), Tf});
      opti.subject_to(result[0] == X(Slice(), k + 1));
      J += result[1];
    }

    // 状态约束（速度限制）
    for (int k = 1; k < N_; ++k) {
      // XY速度限制
      opti.subject_to(-max_vel_xy_ <= X(3, k));  // vx
      opti.subject_to(X(3, k) <= max_vel_xy_);
      opti.subject_to(-max_vel_xy_ <= X(4, k));  // vy
      opti.subject_to(X(4, k) <= max_vel_xy_);
      // 角速度限制
      opti.subject_to(-max_vel_theta_ <= X(5, k));  // omega
      opti.subject_to(X(5, k) <= max_vel_theta_);
    }

    // 控制约束
    for (int k = 0; k < N_; ++k) {
      opti.subject_to(-max_acc_xy_ <= U(0, k));  // ax
      opti.subject_to(U(0, k) <= max_acc_xy_);
      opti.subject_to(-max_acc_xy_ <= U(1, k));  // ay
      opti.subject_to(U(1, k) <= max_acc_xy_);
      opti.subject_to(-max_acc_theta_ <= U(2, k));  // a_omega
      opti.subject_to(U(2, k) <= max_acc_theta_);
    }

    // 画布边界约束 - 暂时禁用，让求解器有更多空间
    // TODO: 在基础问题可行后再启用
    /*
    for (int k = 0; k < N_ + 1; ++k) {
      MX pos = X(Slice(0, 2), k);
      for (int i = 0; i < 4; ++i) {
        MX constraint = A_canvas_(i, 0) * pos(0) + A_canvas_(i, 1) * pos(1);
        opti.subject_to(constraint <= b_canvas_(i));
      }
    }
    */

    // === 7. 障碍物避碰约束（超平面分离）===
    for (int k = 0; k < N_ + 1; ++k) {
      // 计算车辆顶点
      MX x_k = X(0, k);
      MX y_k = X(1, k);
      MX theta_k = X(2, k);

      // 旋转矩阵（符号）
      MX R = MX::vertcat({
        MX::horzcat({MX::cos(theta_k), -MX::sin(theta_k)}),
        MX::horzcat({MX::sin(theta_k),  MX::cos(theta_k)})
      });

      double half_l = vehicle_length_ / 2.0;
      double half_w = vehicle_width_ / 2.0;
      DM V_local = DM(std::vector<std::vector<double>>{
        { half_l,  half_l, -half_l, -half_l},
        { half_w, -half_w, -half_w,  half_w}
      });

      MX V_world = MX::mtimes(R, V_local);
      for (int i = 0; i < 4; ++i) {
        V_world(0, i) = V_world(0, i) + x_k;
        V_world(1, i) = V_world(1, i) + y_k;
      }

      // 对每个障碍物，添加分离超平面
      for (size_t obs_idx = 0; obs_idx < obstacles.size(); ++obs_idx) {
        const auto& obs = obstacles[obs_idx];
        
        // 计算障碍物顶点
        int n_halfspaces = obs.A.size1();
        
        // 超平面参数: lambda (2x1), mu (标量)
        MX lambda = opti.variable(2);
        MX mu = opti.variable();

        // 松弛变量 (slack variables) 用于软约束
        MX slack = opti.variable();
        
        // Slack 必须非负
        opti.subject_to(slack >= 0);
        
        // 在目标函数中惩罚 Slack
        J = J + 10000.0 * slack * slack; 

        // 参数范围约束
        opti.subject_to(-lambda_max_ <= lambda(0));
        opti.subject_to(lambda(0) <= lambda_max_);
        opti.subject_to(-lambda_max_ <= lambda(1));
        opti.subject_to(lambda(1) <= lambda_max_);
        opti.subject_to(-mu_max_ <= mu);
        opti.subject_to(mu <= mu_max_);

        // 法向量非零约束
        opti.subject_to(lambda(0)*lambda(0) + lambda(1)*lambda(1) >= eps_);

        // 从半平面表示推导障碍物顶点
        // 每条边的法向量和偏移存储在(A, b)中
        // 通过求相邻边的交点得到凸包顶点
        double obs_cx = 0, obs_cy = 0;
        std::vector<std::pair<double, double>> obs_vertices;

        for (int i = 0; i < n_halfspaces; ++i) {
          int j = (i + 1) % n_halfspaces;
          
          // 边i: A[i,0]*x + A[i,1]*y = b[i]
          // 边j: A[j,0]*x + A[j,1]*y = b[j]
          double a1 = obs.A(i, 0).scalar();
          double b1 = obs.A(i, 1).scalar();
          double c1 = obs.b(i).scalar();
          
          double a2 = obs.A(j, 0).scalar();
          double b2 = obs.A(j, 1).scalar();
          double c2 = obs.b(j).scalar();
          
          // 求交点 (Cramer's rule)
          double det = a1 * b2 - a2 * b1;
          if (std::abs(det) > 1e-9) {
            double x = (c1 * b2 - c2 * b1) / det;
            double y = (a1 * c2 - a2 * c1) / det;
            obs_vertices.push_back({x, y});
            obs_cx += x;
            obs_cy += y;
          }
        }
        
        if (obs_vertices.empty()) {
          continue;  // 无法计算顶点，跳过
        }
        
        obs_cx /= obs_vertices.size();
        obs_cy /= obs_vertices.size();

        // --- 智能初始猜测 ---
        double alpha = static_cast<double>(k) / N_;
        double ref_x = (1 - alpha) * start_state[0] + alpha * goal_state[0];
        double ref_y = (1 - alpha) * start_state[1] + alpha * goal_state[1];

        double dx = ref_x - obs_cx;
        double dy = ref_y - obs_cy;
        double norm = std::sqrt(dx*dx + dy*dy);
        
        if (norm < 1e-3) { dx = 1.0; dy = 1.0; norm = 1.414; }
        
        std::vector<double> lambda_init = {dx / norm, dy / norm};

        double max_proj_obs = -1e9;
        for (const auto& vert : obs_vertices) {
             double proj = lambda_init[0] * vert.first + lambda_init[1] * vert.second;
             if (proj > max_proj_obs) max_proj_obs = proj;
        }
        
        double mu_init = max_proj_obs + 0.1; 

        opti.set_initial(lambda, DM(lambda_init));
        opti.set_initial(mu, mu_init);
        opti.set_initial(slack, 1.0);

        // 车辆顶点约束
        for (int v_idx = 0; v_idx < 4; ++v_idx) {
          MX v = V_world(Slice(), v_idx);
          opti.subject_to(lambda(0)*v(0) + lambda(1)*v(1) >= mu + eps_ - slack);
        }

        // 障碍物顶点约束
        for (const auto& vert : obs_vertices) {
          opti.subject_to(lambda(0)*vert.first + lambda(1)*vert.second <= mu - eps_ + slack);
        }
      }
    }

    // === 8. 目标函数 ===
    J += weight_time_ * Tf;
    opti.minimize(J);

    // === 9. 初始猜测（使用 Warm Start 或回退到线性插值）===
    if (X_init.size1() == 6 && X_init.size2() == static_cast<casadi_int>(N_ + 1)) {
      // 使用 HybridAStar warm start
      opti.set_initial(X, X_init);
      RCLCPP_INFO(node->get_logger(), "Using HybridAStar warm start for X");
    } else {
      // 回退：线性插值
      DM X_fallback = DM::zeros(6, N_ + 1);
      for (int k = 0; k <= N_; ++k) {
        double alpha = static_cast<double>(k) / N_;
        X_fallback(0, k) = (1 - alpha) * start_state[0] + alpha * goal_state[0];
        X_fallback(1, k) = (1 - alpha) * start_state[1] + alpha * goal_state[1];
        X_fallback(2, k) = (1 - alpha) * start_state[2] + alpha * goal_state[2];
        X_fallback(3, k) = 0.0;  // vx
        X_fallback(4, k) = 0.0;  // vy
        X_fallback(5, k) = 0.0;  // omega
      }
      opti.set_initial(X, X_fallback);
      RCLCPP_INFO(node->get_logger(), "Using linear interpolation for X (fallback)");
    }
    
    if (U_init.size1() == 3 && U_init.size2() == static_cast<casadi_int>(N_)) {
      opti.set_initial(U, U_init);
    } else {
      opti.set_initial(U, DM::zeros(3, N_));
    }
    opti.set_initial(Tf, 10.0);

    // === 10. 求解器配置 ===
    Dict opts;
    opts["expand"] = true;
    opts["ipopt.max_iter"] = static_cast<int>(solver_max_iter_);
    opts["ipopt.tol"] = solver_tol_;
    opts["ipopt.print_level"] = 3;
    opts["ipopt.max_cpu_time"] = solver_time_limit_;
    opts["print_time"] = true;

    opts["error_on_fail"] = false;
    opti.solver("ipopt", opts);

    // === 11. 求解 ===
    RCLCPP_INFO(node->get_logger(), "Starting optimization...");
    OptiSol sol = opti.solve();

    // === 12. 提取结果 ===
    std::string status = sol.stats().at("return_status");
    if (status != "Solve_Succeeded") {
        RCLCPP_WARN(node->get_logger(), "Optimization failed status: %s", status.c_str());
        // Check if we can still use the result if it's "Solved To Acceptable Level"?
        if (status != "Solved_To_Acceptable_Level") {
             return false;
        }
    }

    X_out = sol.value(X);
    U_out = sol.value(U);
    tf_out = sol.value(Tf).scalar();

    RCLCPP_INFO(node->get_logger(), 
      "Optimization succeeded! Cost: %.3f, Time: %.2f s",
      sol.value(J).scalar(), tf_out);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), 
      "Optimization failed with exception: %s", e.what());
    return false;
  }
}

nav_msgs::msg::Path HyperplanePlanner::trajectoryToPath(
  const casadi::DM& X,
  double /* tf */,
  const geometry_msgs::msg::PoseStamped& start)
{
  nav_msgs::msg::Path path;
  path.header.stamp = start.header.stamp;
  path.header.frame_id = global_frame_;

  int n_points = X.size2();
  for (int i = 0; i < n_points; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = X(0, i).scalar();
    pose.pose.position.y = X(1, i).scalar();
    pose.pose.position.z = 0.0;

    double theta = X(2, i).scalar();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
  }

  return path;
}

#endif  // DCS_WITH_CASADI

}  // namespace dcs_nav_plugin
