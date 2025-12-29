#include "dcs_nav_plugin/mpc/mpc_solver_casadi.hpp"
#include <chrono>
#include <iostream>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace casadi;

namespace dcs_nav_plugin
{

MpcSolverCasadi::MpcSolverCasadi()
{
}

MpcSolverCasadi::~MpcSolverCasadi()
{
}

void MpcSolverCasadi::configure(const MpcConfig & config)
{
  config_ = config;
  initialized_ = false;
  first_run_ = true;
}

void MpcSolverCasadi::initialize()
{
  if (initialized_) return;

  try {
    buildProblem();
    initialized_ = true;
    first_run_ = true; // Reset warm start on re-init
    std::cout << "[MpcSolverCasadi] Solver initialized successfully." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[MpcSolverCasadi] Initialization failed: " << e.what() << std::endl;
  }
}

void MpcSolverCasadi::buildProblem()
{
  opti_ = casadi::Opti();

  int N = config_.N;
  int n_x = 3; 
  int n_u = 3; 
  
  X_ = opti_.variable(n_x, N+1);
  U_ = opti_.variable(n_u, N);
  int K = max_obstacles_;
  eps_slack_ = opti_.variable(K, N);
  
  // Parameters
  P_x0_ = opti_.parameter(n_x);
  P_ref_ = opti_.parameter(n_x, N+1);
  
  // Obstacle Params
  P_obs_Vx_ = opti_.parameter(K, max_obs_vertices_);
  P_obs_Vy_ = opti_.parameter(K, max_obs_vertices_);
  P_obs_valid_ = opti_.parameter(K, 1);

  // Objective
  MX J = 0;
  for(int k=0; k<N; ++k) {
    MX err_x = X_(0,k) - P_ref_(0,k);
    MX err_y = X_(1,k) - P_ref_(1,k);
    MX err_theta = X_(2,k) - P_ref_(2,k);
    
    J += config_.Q_x * pow(err_x, 2) + 
         config_.Q_y * pow(err_y, 2) + 
         config_.Q_theta * pow(err_theta, 2);
         
    J += config_.R_v * (pow(U_(0,k), 2) + pow(U_(1,k), 2)) + 
         config_.R_omega * pow(U_(2,k), 2);
  }
  J += config_.Q_x * pow(X_(0,N) - P_ref_(0,N), 2) + 
       config_.Q_y * pow(X_(1,N) - P_ref_(1,N), 2) + 
       config_.Q_theta * pow(X_(2,N) - P_ref_(2,N), 2);
       
  // Initial Cond
  opti_.subject_to(X_(Slice(), 0) == P_x0_);
  
  // Dynamics using RK4 Integration
  double dt = config_.dt;
  
  // ODE function: dx/dt = f(x, u)
  // x = [x, y, theta], u = [vx, vy, omega]
  // Returns [dx/dt, dy/dt, dtheta/dt]
  auto ode = [](MX x, MX y, MX th, MX vx, MX vy, MX w) -> std::tuple<MX, MX, MX> {
    MX dx_dt = vx * cos(th) - vy * sin(th);
    MX dy_dt = vx * sin(th) + vy * cos(th);
    MX dth_dt = w;
    return {dx_dt, dy_dt, dth_dt};
  };
  
  for(int k=0; k<N; ++k) {
    MX x_k = X_(0, k);
    MX y_k = X_(1, k);
    MX th_k = X_(2, k);
    MX vx = U_(0, k);
    MX vy = U_(1, k);
    MX w = U_(2, k);
    
    // RK4 Step
    auto [k1_x, k1_y, k1_th] = ode(x_k, y_k, th_k, vx, vy, w);
    auto [k2_x, k2_y, k2_th] = ode(x_k + 0.5*dt*k1_x, y_k + 0.5*dt*k1_y, th_k + 0.5*dt*k1_th, vx, vy, w);
    auto [k3_x, k3_y, k3_th] = ode(x_k + 0.5*dt*k2_x, y_k + 0.5*dt*k2_y, th_k + 0.5*dt*k2_th, vx, vy, w);
    auto [k4_x, k4_y, k4_th] = ode(x_k + dt*k3_x, y_k + dt*k3_y, th_k + dt*k3_th, vx, vy, w);
    
    MX x_next = x_k + (dt/6.0)*(k1_x + 2*k2_x + 2*k3_x + k4_x);
    MX y_next = y_k + (dt/6.0)*(k1_y + 2*k2_y + 2*k3_y + k4_y);
    MX th_next = th_k + (dt/6.0)*(k1_th + 2*k2_th + 2*k3_th + k4_th);
    
    opti_.subject_to(X_(0, k+1) == x_next);
    opti_.subject_to(X_(1, k+1) == y_next);
    opti_.subject_to(X_(2, k+1) == th_next);
    
    // Constraints
    opti_.subject_to(vx*vx + vy*vy <= config_.v_max*config_.v_max);
    opti_.subject_to(opti_.bounded(-config_.omega_max, w, config_.omega_max));
  }
  
  // Obstacle Constraints with SHT and Warm Start Hints
  // We need to define A and b as variables but maybe store them in a way accessible for warm start
  // For simplicity, we just declare them here and let the optimizer handle them.
  // Ideally, A and b should be initialized based on geometry.
  
  MX V_car_local = MX::zeros(2, 4);
  // Box approx
  double half_l = config_.car_length / 2.0;
  double half_w = config_.car_width / 2.0;
  V_car_local(0,0) = half_l;  V_car_local(1,0) = half_w;
  V_car_local(0,1) = -half_l; V_car_local(1,1) = half_w;
  V_car_local(0,2) = -half_l; V_car_local(1,2) = -half_w;
  V_car_local(0,3) = half_l;  V_car_local(1,3) = -half_w;
  
  A_sht_vars_.clear();
  b_sht_vars_.clear();
  A_sht_vars_.reserve(K * N);
  b_sht_vars_.reserve(K * N);
  
  for(int j=0; j<K; ++j) {
      for(int k=0; k<N; ++k) {
          MX A = opti_.variable(2, 1);
          MX b = opti_.variable(1);
          MX eps = eps_slack_(j, k);
          
          // 存储变量引用用于 Warm Start
          A_sht_vars_.push_back(A);
          b_sht_vars_.push_back(b);
          
          J += config_.lambda_slack * pow(eps, 2) * P_obs_valid_(j); 
          opti_.subject_to(eps >= 0);
          
          // NORMALIZATION CONSTRAINT (CRITICAL FIX)
          // |A| <= 1. This ensures that 'margin' represents true physical distance.
          // Without this, solver can scale A up (e.g. A*100, b*100) to make 'margin' negligible.
          opti_.subject_to(mtimes(A.T(), A) <= 1.0 + 1e-4); 

          opti_.subject_to(mtimes(A.T(), A) >= config_.sht_epsilon);
          
          // Car Side
          MX th = X_(2,k);
          MX tx = X_(0,k);
          MX ty = X_(1,k);
          MX R = MX::zeros(2,2);
          R(0,0)=cos(th); R(0,1)=-sin(th);
          R(1,0)=sin(th); R(1,1)=cos(th);
          
          MX V_car_world = mtimes(R, V_car_local) + repmat(vertcat(tx, ty), 1, 4);
          MX dist_car = mtimes(A.T(), V_car_world);
          
          // Obstacle Side
          MX V_obs_x = P_obs_Vx_(j, Slice());
          MX V_obs_y = P_obs_Vy_(j, Slice());
          MX V_obs = vertcat(V_obs_x, V_obs_y);
          MX dist_obs = mtimes(A.T(), V_obs);
          
          // Big-M 方法：当障碍物无效时，约束自动满足
          double M = 1000.0;
          MX valid_mx = P_obs_valid_(j);
          
          // 车辆约束：dist_car >= b + margin - eps (+ M*(1-valid))
          opti_.subject_to( dist_car >= (b + config_.margin - eps - M*(1-valid_mx)) );
          
          // 障碍物约束：dist_obs <= b - margin (+ M*(1-valid))
          opti_.subject_to( dist_obs <= (b - config_.margin + M*(1-valid_mx)) );
      }
  }

  opti_.minimize(J);

  // IPOPT Solver Options - Conservative Performance Optimization
  Dict opts;
  opts["ipopt.print_level"] = 5; // Enable full internal logging as requested
  opts["print_time"] = 0;
  opts["expand"] = true;
  
  // Iteration and convergence limits
  opts["ipopt.max_iter"] = 100;              // 限制最大迭代次数
  opts["ipopt.tol"] = 1e-3;                  // 放宽收敛精度
  opts["ipopt.acceptable_tol"] = 0.01;       // 可接受解的容差
  opts["ipopt.acceptable_iter"] = 15;        // 允许提前终止的迭代次数
  
  opti_.solver("ipopt", opts);
}

MpcSolution MpcSolverCasadi::solve(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_twist,
    const std::vector<geometry_msgs::msg::PoseStamped> & ref_traj,
    const std::vector<Polygon> & obstacles)
{
  (void)current_twist; 
  MpcSolution solution;
  auto start = std::chrono::high_resolution_clock::now();

  try {
    std::vector<double> x0 = {
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      tf2::getYaw(current_pose.pose.orientation)
    };
    opti_.set_value(P_x0_, x0);
    
    // 2. Set Reference Trajectory
    int N = config_.N;
    DM ref_dm = DM::zeros(3, N+1);
    for(int k = 0; k <= N && k < (int)ref_traj.size(); ++k) {
        ref_dm(0, k) = ref_traj[k].pose.position.x;
        ref_dm(1, k) = ref_traj[k].pose.position.y;
        ref_dm(2, k) = tf2::getYaw(ref_traj[k].pose.orientation);
    }
    // Pad with last pose if ref_traj is shorter
    for(int k = (int)ref_traj.size(); k <= N; ++k) {
        if (!ref_traj.empty()) {
            ref_dm(0, k) = ref_traj.back().pose.position.x;
            ref_dm(1, k) = ref_traj.back().pose.position.y;
            ref_dm(2, k) = tf2::getYaw(ref_traj.back().pose.orientation);
        }
    }
    opti_.set_value(P_ref_, ref_dm);
    
    // 3. Set Obstacles
    // Convert std::vector<Polygon> to CasADi DM
    int K = max_obstacles_;
    int V_max = max_obs_vertices_;
    
    int n_obs = std::min((int)obstacles.size(), K);
    
    DM obs_vx = DM::zeros(K, V_max);
    DM obs_vy = DM::zeros(K, V_max);
    DM obs_valid = DM::zeros(K, 1);
    
    for(int j=0; j<n_obs; ++j) {
        const auto & poly = obstacles[j];
        int n_pts = std::min((int)poly.size(), V_max);
        
        if (n_pts < 3) continue; 
        
        obs_valid(j) = 1.0;
        
        for(int i=0; i<n_pts; ++i) {
            obs_vx(j, i) = poly[i].x;
            obs_vy(j, i) = poly[i].y;
        }
        
        for(int i=n_pts; i<V_max; ++i) {
            obs_vx(j, i) = poly[n_pts-1].x;
            obs_vy(j, i) = poly[n_pts-1].y;
        }
    }
    
    opti_.set_value(P_obs_Vx_, obs_vx);
    opti_.set_value(P_obs_Vy_, obs_vy);
    opti_.set_value(P_obs_valid_, obs_valid);
    
    // ============================================
    // Warm Start Loop with Retry
    // ============================================
    
    bool use_warm_start = !first_run_ && !last_u_opt_.is_empty() && !last_x_opt_.is_empty();
    bool solve_successful = false;
    
    // We try at most 2 times: 
    // 1. First attempt (Warm if available, else Cold)
    // 2. Second attempt (Cold, only if first was Warm and failed)
    
    for (int attempt = 0; attempt < 2; ++attempt) {
        
        // Setup Initial Guess
        if (use_warm_start) {
             // Warm Start: Shift previous solution
            DM u_guess = DM::zeros(3, N);
            u_guess(Slice(), Slice(0, N-1)) = last_u_opt_(Slice(), Slice(1, N));
            u_guess(Slice(), N-1) = last_u_opt_(Slice(), N-1); 
            opti_.set_initial(U_, u_guess);
            
            DM x_guess = DM::zeros(3, N+1);
            x_guess(Slice(), Slice(0, N)) = last_x_opt_(Slice(), Slice(1, N+1));
            x_guess(Slice(), N) = last_x_opt_(Slice(), N); 
            opti_.set_initial(X_, x_guess);
        } else {
             // Cold Start: Use Reference Trajectory
            opti_.set_initial(X_, ref_dm);
            DM u_init = DM::zeros(3, N);
            opti_.set_initial(U_, u_init);
        }
        
        // SHT Geometry Warm Start (Always apply geometric hint if possible, purely based on kinematics)
        // ... (Same as before)
         for(int j = 0; j < n_obs; ++j) {
            const auto& poly = obstacles[j];
            if (poly.size() < 3) continue;
            
            double cx = 0.0, cy = 0.0;
            for (const auto& pt : poly) { cx += pt.x; cy += pt.y; }
            cx /= poly.size();
            cy /= poly.size();
            
            double dx = x0[0] - cx; 
            double dy = x0[1] - cy;
            double norm = std::hypot(dx, dy);
            
            if (norm > 0.01) {
                double a_x = dx / norm;
                double a_y = dy / norm;
                double init_b = (a_x * (x0[0] + cx) / 2.0 + a_y * (x0[1] + cy) / 2.0);
                
                for(int k = 0; k < N; ++k) {
                    int idx = j * N + k;
                    if (idx < static_cast<int>(A_sht_vars_.size())) {
                        opti_.set_initial(A_sht_vars_[idx], std::vector<double>{a_x, a_y});
                        opti_.set_initial(b_sht_vars_[idx], init_b);
                    }
                }
            }
        }
        
        // Try Solve
        try {
            casadi::OptiSol sol = opti_.solve(); 
            
            // If we are here, solve succeeded
            solution.success = true;
            
            // ... Extract Results ...
            DM u_opt = sol.value(U_);
            solution.cmd_vel.linear.x = double(u_opt(0,0));
            solution.cmd_vel.linear.y = double(u_opt(1,0));
            solution.cmd_vel.angular.z = double(u_opt(2,0));
            
            try {
                DM x_opt = sol.value(X_); 
                for(int k=0; k<=N; ++k) { 
                     geometry_msgs::msg::PoseStamped p;
                     p.header.frame_id = "odom"; 
                     p.pose.position.x = double(x_opt(0,k));
                     p.pose.position.y = double(x_opt(1,k));
                     p.pose.position.z = 0.0;
                     double th = double(x_opt(2,k));
                     tf2::Quaternion q;
                     q.setRPY(0, 0, th);
                     p.pose.orientation = tf2::toMsg(q);
                     solution.predicted_traj.push_back(p);
                }
                // Cache Solution
                last_u_opt_ = u_opt;
                last_x_opt_ = x_opt;
                first_run_ = false; // Successfully ran at least once
            } catch(const std::exception& e) {
                // If extraction fails, it's weird but not a solver failure
            }

            // Check Slack
            DM eps_opt = sol.value(eps_slack_); 
            double max_eps = 0.0;
            for(int j=0; j<n_obs; ++j) {
                for(int k=0; k<N; ++k) {
                    double val = double(eps_opt(j,k));
                    if(val > max_eps) max_eps = val;
                }
            }
            solution.max_slack = max_eps;
            
            // Stats
            try {
                if(sol.stats().count("iter_count")) solution.iter_count = static_cast<int>(sol.stats().at("iter_count"));
                if(sol.stats().count("return_status")) solution.return_status = static_cast<std::string>(sol.stats().at("return_status"));
            } catch (...) {}
            
            solve_successful = true;
            break; // Break retry loop
            
        } catch (const std::exception &e) {
            // Failure!
            if (use_warm_start) {
                // If we failed with Warm Start, we MUST try again with Cold Start
                std::cerr << "[MPC] Warm start failed (" << e.what() << "), retrying with Cold Start..." << std::endl;
                use_warm_start = false; // Next iteration will be cold
                first_run_ = true; // Reset flag for logic consistency
                continue; // Continue to next attempt
            } else {
                // If we failed with Cold Start (or second attempt), we are done
                solution.success = false;
                solution.return_status = "Exception: " + std::string(e.what());
                first_run_ = true; // Reset for next frame safety
                break;
            }
        }
    } // End Retry Loop

  } catch (const std::exception &e) {
    // Outer catch for logic errors before solve
    solution.success = false;
    solution.return_status = "Exception: " + std::string(e.what());
    first_run_ = true; 
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  solution.solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
  
  return solution;
}

std::string MpcSolverCasadi::getDebugString(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped> & ref_traj,
    const std::vector<Polygon> & obstacles)
{
  std::stringstream ss;
  ss.precision(4);
  ss << std::fixed;

  ss << "========== MPC 优化问题 ==========\n";
  
  // 1. Parameters & Weights
  ss << "[参数] N=" << config_.N << ", dt=" << config_.dt << "\n";
  ss << "[权重] Q_x=" << config_.Q_x << ", Q_y=" << config_.Q_y << ", Q_theta=" << config_.Q_theta << "\n";
  ss << "[权重] R_v=" << config_.R_v << ", R_w=" << config_.R_omega << ", 松弛限制=" << config_.lambda_slack << "\n";
  ss << "[约束] v_max=" << config_.v_max << ", w_max=" << config_.omega_max << ", 边距=" << config_.margin << "\n";
  
  // 2. Initial State
  double x0 = current_pose.pose.position.x;
  double y0 = current_pose.pose.position.y;
  double th0 = tf2::getYaw(current_pose.pose.orientation);
  ss << "[初始状态] x=" << x0 << ", y=" << y0 << ", theta=" << th0 << "\n";

  // 3. Reference Trajectory
  ss << "[参考轨迹] 总点数: " << ref_traj.size() << "\n";
  for(size_t i=0; i<ref_traj.size(); ++i) {
      ss << "  [" << i << "] x=" << ref_traj[i].pose.position.x 
         << ", y=" << ref_traj[i].pose.position.y 
         << ", th=" << tf2::getYaw(ref_traj[i].pose.orientation) << "\n";
  }
  
  // 4. Obstacles
  ss << "[障碍物] 数量: " << obstacles.size() << " (最大: " << max_obstacles_ << ")\n";
  for(size_t i=0; i<obstacles.size(); ++i) {
     ss << "  障碍[" << i << "] 顶点数=" << obstacles[i].size() << ": ";
     for(const auto& pt : obstacles[i]) {
         ss << "(" << pt.x << "," << pt.y << ") ";
     }
     ss << "\n";
  }
  
  ss << "==============================================\n";
  return ss.str();
}

std::string MpcSolverCasadi::getResultDebugString(const MpcSolution& sol)
{
  std::stringstream ss;
  ss.precision(4);
  ss << std::fixed;
  
  ss << "========== MPC 求解结果 (IPOPT) ==========\n";
  ss << "状态: " << (sol.success ? "成功" : "失败") << " (" << sol.return_status << ")\n";
  ss << "耗时: " << sol.solve_time_ms << " ms\n";
  ss << "迭代次数: " << sol.iter_count << "\n";
  ss << "最大松弛变量: " << sol.max_slack << "\n";
  ss << "------------------------------------------\n";
  ss << "控制输出 [Vx, Vy, Omega]:\n";
  ss << "  [" << sol.cmd_vel.linear.x << ", " 
     << sol.cmd_vel.linear.y << ", " 
     << sol.cmd_vel.angular.z << "]\n";
  
  ss << "------------------------------------------\n";
  ss << "优化轨迹 (前5点/共" << sol.predicted_traj.size() << "点):\n";
  
  int print_points = std::min((int)sol.predicted_traj.size(), 5);
  for(int i=0; i<print_points; ++i) {
      const auto& p = sol.predicted_traj[i].pose;
      ss << "  [" << i << "] x=" << p.position.x 
         << ", y=" << p.position.y 
         << ", th=" << tf2::getYaw(p.orientation) << "\n";
  }
  ss << "==========================================\n";
  
  return ss.str();
}

}  // namespace dcs_nav_plugin
