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
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
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
    node, name + ".max_vel_x", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".max_vel_x", max_vel_x_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".max_vel_theta", max_vel_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_acc_x", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".max_acc_x", max_acc_x_);

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

#ifdef DCS_WITH_CASADI
  RCLCPP_INFO(node->get_logger(), 
    "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // 1. 提取起点和终点状态
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_theta = tf2::getYaw(start.pose.orientation);

  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double goal_theta = tf2::getYaw(goal.pose.orientation);

  // 状态向量: [x, y, theta, v, omega]
  std::vector<double> start_state = {start_x, start_y, start_theta, 0.0, 0.0};
  std::vector<double> goal_state = {goal_x, goal_y, goal_theta, 0.0, 0.0};

  // 2. 提取障碍物
  std::vector<Obstacle> obstacles = extractObstacles();
  RCLCPP_INFO(node->get_logger(), "Extracted %zu obstacles", obstacles.size());

  // 3. 求解优化问题
  casadi::DM X_opt, U_opt;
  double tf_opt;
  
  bool success = solveOptimization(start_state, goal_state, obstacles, 
                                   X_opt, U_opt, tf_opt);

  if (success) {
    RCLCPP_INFO(node->get_logger(), 
      "Optimization succeeded! Time: %.2f seconds", tf_opt);
    global_path = trajectoryToPath(X_opt, tf_opt, start);
  } else {
    RCLCPP_WARN(node->get_logger(), "Optimization failed!");
  }
#else
  RCLCPP_ERROR(node->get_logger(), "CasADi not available!");
#endif

  return global_path;
}

#ifdef DCS_WITH_CASADI

std::vector<Obstacle> HyperplanePlanner::extractObstacles()
{
  std::vector<Obstacle> obstacles;

  // 简化版：从代价地图中提取占据栅格，聚类成矩形障碍物
  // 这里为演示，创建几个示例矩形障碍物
  
  // 扫描代价地图，查找致命障碍物
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  std::vector<std::pair<double, double>> obstacle_cells;
  
  for (unsigned int i = 0; i < size_x; ++i) {
    for (unsigned int j = 0; j < size_y; ++j) {
      unsigned char cost = costmap_->getCost(i, j);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy;
        costmap_->mapToWorld(i, j, wx, wy);
        obstacle_cells.push_back({wx, wy});
      }
    }
  }

  // 简单聚类：将相邻的障碍物栅格合并成矩形
  // 为简化实现，这里每 10 个障碍物栅格创建一个矩形
  const size_t cluster_size = 20;
  for (size_t i = 0; i < obstacle_cells.size(); i += cluster_size) {
    if (obstacle_cells.empty()) break;
    
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    size_t end = std::min(i + cluster_size, obstacle_cells.size());
    for (size_t j = i; j < end; ++j) {
      min_x = std::min(min_x, obstacle_cells[j].first);
      max_x = std::max(max_x, obstacle_cells[j].first);
      min_y = std::min(min_y, obstacle_cells[j].second);
      max_y = std::max(max_y, obstacle_cells[j].second);
    }

    // 膨胀
    min_x -= obstacle_inflation_;
    max_x += obstacle_inflation_;
    min_y -= obstacle_inflation_;
    max_y += obstacle_inflation_;

    // 创建矩形约束: [上, 右, 下, 左]
    casadi::DM A = casadi::DM(std::vector<std::vector<double>>{
      {0, 1}, {1, 0}, {0, -1}, {-1, 0}
    });
    casadi::DM b = casadi::DM(std::vector<double>{
      max_y, max_x, -min_y, -min_x
    });

    obstacles.emplace_back(A, b);
  }

  return obstacles;
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

bool HyperplanePlanner::solveOptimization(
  const std::vector<double>& start_state,
  const std::vector<double>& goal_state,
  const std::vector<Obstacle>& obstacles,
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

    // === 2. 定义决策变量 ===
    // 状态: X = [x, y, theta, v, omega]^T (5 x N+1)
    MX X = opti.variable(5, N_ + 1);
    // 控制: U = [a_x, a_omega]^T (2 x N)
    MX U = opti.variable(2, N_);
    // 总时间: Tf (标量)
    MX Tf = opti.variable();

    // === 3. 定义符号变量（用于动力学）===
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX theta = MX::sym("theta");
    MX v = MX::sym("v");
    MX omega = MX::sym("omega");
    MX state = MX::vertcat({x, y, theta, v, omega});

    MX a_x = MX::sym("a_x");
    MX a_omega = MX::sym("a_omega");
    MX control = MX::vertcat({a_x, a_omega});

    // === 4. 差速驱动动力学模型 ===
    // dx/dt = v * cos(theta)
    // dy/dt = v * sin(theta)
    // dtheta/dt = omega
    // dv/dt = a_x
    // domega/dt = a_omega
    MX x_dot = MX::vertcat({
      v * MX::cos(theta),
      v * MX::sin(theta),
      omega,
      a_x,
      a_omega
    });

    // 代价函数（阶段性代价）
    MX stage_cost = weight_smooth_ * (a_x * a_x + a_omega * a_omega);

    Function dynamics("f", {state, control}, {x_dot, stage_cost});

    // === 5. RK4 积分器 ===
    double h = 1.0 / N_;  // 归一化时间步长
    double dt = h / M_;   // 子步长

    MX X_k = MX::sym("X_k", 5);
    MX U_k = MX::sym("U_k", 2);
    MX X_next = X_k;
    MX Q = 0;

    for (int m = 0; m < M_; ++m) {
      auto k1 = dynamics(std::vector<MX>{X_next, U_k});
      auto k2 = dynamics(std::vector<MX>{X_next + dt/2 * k1[0], U_k});
      auto k3 = dynamics(std::vector<MX>{X_next + dt/2 * k2[0], U_k});
      auto k4 = dynamics(std::vector<MX>{X_next + dt * k3[0], U_k});
      
      X_next = X_next + dt/6 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
      Q = Q + dt/6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
    }

    Function integrator("F", {X_k, U_k, Tf}, {X_next * Tf, Q * Tf});

    // === 6. 添加约束 ===
    
    // 初始状态约束
    opti.subject_to(X(Slice(), 0) == DM(start_state));
    
    // 终点状态约束
    opti.subject_to(X(Slice(), N_) == DM(goal_state));

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

    // 状态约束
    for (int k = 1; k < N_; ++k) {  // 跳过起点和终点
      // 速度限制
      opti.subject_to(-max_vel_x_ <= X(3, k));
      opti.subject_to(X(3, k) <= max_vel_x_);
      opti.subject_to(-max_vel_theta_ <= X(4, k));
      opti.subject_to(X(4, k) <= max_vel_theta_);
    }

    // 控制约束
    for (int k = 0; k < N_; ++k) {
      opti.subject_to(-max_acc_x_ <= U(0, k));
      opti.subject_to(U(0, k) <= max_acc_x_);
      opti.subject_to(-max_acc_theta_ <= U(1, k));
      opti.subject_to(U(1, k) <= max_acc_theta_);
    }

    // 画布边界约束
    for (int k = 0; k < N_ + 1; ++k) {
      // 简化：只约束中心点在边界内
      // 更严格的做法是约束所有顶点
      MX pos = X(Slice(0, 2), k);  // [x, y]
      
      // A_canvas * pos <= b_canvas
      for (int i = 0; i < 4; ++i) {
        MX constraint = A_canvas_(i, 0) * pos(0) + A_canvas_(i, 1) * pos(1);
        opti.subject_to(constraint <= b_canvas_(i));
      }
    }

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

        // 初始猜测
        opti.set_initial(lambda, DM(std::vector<double>{1, 1}));
        opti.set_initial(mu, 1);

        // 参数范围约束
        opti.subject_to(-lambda_max_ <= lambda(0));
        opti.subject_to(lambda(0) <= lambda_max_);
        opti.subject_to(-lambda_max_ <= lambda(1));
        opti.subject_to(lambda(1) <= lambda_max_);
        opti.subject_to(-mu_max_ <= mu);
        opti.subject_to(mu <= mu_max_);

        // 法向量非零约束
        opti.subject_to(lambda(0)*lambda(0) + lambda(1)*lambda(1) >= eps_);

        // 车辆顶点在超平面一侧: lambda^T * v > mu
        for (int v_idx = 0; v_idx < 4; ++v_idx) {
          MX v = V_world(Slice(), v_idx);
          opti.subject_to(lambda(0)*v(0) + lambda(1)*v(1) >= mu + eps_);
        }

        // 障碍物顶点在超平面另一侧: A*p <= b => lambda^T * p < mu
        // 简化：假设障碍物是矩形，从 A*p<=b 计算顶点
        // 实际上这需要专门的顶点计算函数
        // 这里我们跳过精确计算，使用障碍物中心作为代表点
        
        // 从半平面表示推导障碍物边界
        // 对于矩形: [上, 右, 下, 左]
        if (n_halfspaces == 4) {
          double y_max = obs.b(0).scalar();  // 上
          double x_max = obs.b(1).scalar();  // 右
          double y_min = -obs.b(2).scalar(); // 下
          double x_min = -obs.b(3).scalar(); // 左

          // 四个顶点
          std::vector<std::pair<double, double>> obs_vertices = {
            {x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}
          };

          for (const auto& vert : obs_vertices) {
            opti.subject_to(lambda(0)*vert.first + lambda(1)*vert.second <= mu - eps_);
          }
        }
      }
    }

    // === 8. 目标函数 ===
    J += weight_time_ * Tf;
    opti.minimize(J);

    // === 9. 初始猜测 ===
    // 线性插值
    DM X_init = DM::zeros(5, N_ + 1);
    for (int k = 0; k <= N_; ++k) {
      double alpha = static_cast<double>(k) / N_;
      X_init(0, k) = (1 - alpha) * start_state[0] + alpha * goal_state[0];
      X_init(1, k) = (1 - alpha) * start_state[1] + alpha * goal_state[1];
      X_init(2, k) = (1 - alpha) * start_state[2] + alpha * goal_state[2];
      X_init(3, k) = 0.0;
      X_init(4, k) = 0.0;
    }
    opti.set_initial(X, X_init);
    opti.set_initial(U, DM::zeros(2, N_));
    opti.set_initial(Tf, 10.0);

    // === 10. 求解器配置 ===
    Dict opts;
    opts["expand"] = true;
    opts["ipopt.max_iter"] = static_cast<int>(solver_max_iter_);
    opts["ipopt.tol"] = solver_tol_;
    opts["ipopt.print_level"] = 3;
    opts["ipopt.max_cpu_time"] = solver_time_limit_;
    opts["print_time"] = true;

    opti.solver("ipopt", opts);

    // === 11. 求解 ===
    RCLCPP_INFO(node->get_logger(), "Starting optimization...");
    OptiSol sol = opti.solve();

    // === 12. 提取结果 ===
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
