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
}

void MpcSolverCasadi::initialize()
{
  if (initialized_) return;

  try {
    buildProblem();
    initialized_ = true;
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
  
  // Dynamics
  double dt = config_.dt;
  for(int k=0; k<N; ++k) {
    MX th = X_(2,k);
    MX vx = U_(0,k);
    MX vy = U_(1,k);
    MX w = U_(2,k);
    
    MX dx = (vx * cos(th) - vy * sin(th)) * dt;
    MX dy = (vx * sin(th) + vy * cos(th)) * dt;
    MX dth = w * dt;
    
    opti_.subject_to(X_(0, k+1) == X_(0,k) + dx);
    opti_.subject_to(X_(1, k+1) == X_(1,k) + dy);
    opti_.subject_to(X_(2, k+1) == X_(2,k) + dth);
    
    // Bounds
    opti_.subject_to(opti_.bounded(-config_.v_max, vx, config_.v_max));
    opti_.subject_to(opti_.bounded(-config_.v_max, vy, config_.v_max));
    opti_.subject_to(opti_.bounded(-config_.omega_max, w, config_.omega_max));
  }
  
  // SHT Constraints
  double L = config_.car_length;
  double W = config_.car_width;
  DM V_car_local = DM::zeros(2, 4);
  V_car_local(0,0)=L/2; V_car_local(1,0)=-W/2; // CasADi uses generalized indexing? No, (row, col)
  // Let's rely on standard: (0,0)=x, (1,0)=y ...
  V_car_local(0,0)=L/2; V_car_local(1,0)=W/2;
  V_car_local(0,1)=L/2; V_car_local(1,1)=-W/2;
  V_car_local(0,2)=-L/2; V_car_local(1,2)=-W/2;
  V_car_local(0,3)=-L/2; V_car_local(1,3)=W/2;
  
  for(int j=0; j<K; ++j) {
      for(int k=0; k<N; ++k) {
          MX A = opti_.variable(2, 1);
          MX b = opti_.variable(1);
          MX eps = eps_slack_(j, k);
          
          J += config_.lambda_slack * pow(eps, 2) * P_obs_valid_(j); 
          opti_.subject_to(eps >= 0);
          
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
          // valid=1: 正常约束, valid=0: 约束被放松
          double M = 1000.0;
          MX valid_mx = P_obs_valid_(j);
          
          // 车辆约束：dist_car >= b + margin - eps (+ M*(1-valid))
          opti_.subject_to( dist_car >= (b + config_.margin - eps - M*(1-valid_mx)) );
          
          // 障碍物约束：dist_obs <= b - margin (+ M*(1-valid))
          opti_.subject_to( dist_obs <= (b - config_.margin + M*(1-valid_mx)) );
      }
  }

  opti_.minimize(J);

  Dict opts;
  opts["ipopt.print_level"] = 0;
  opts["print_time"] = 0;
  opts["expand"] = true;
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
    
    // Debug: 打印参考轨迹前3个点和初始状态
    std::cerr << "[MPC调试] 初始状态: x=" << x0[0] << ", y=" << x0[1] << ", θ=" << x0[2] << std::endl;
    std::cerr << "[MPC调试] 参考轨迹 [0]: x=" << double(ref_dm(0,0)) << ", y=" << double(ref_dm(1,0)) << ", θ=" << double(ref_dm(2,0)) << std::endl;
    std::cerr << "[MPC调试] 参考轨迹 [1]: x=" << double(ref_dm(0,1)) << ", y=" << double(ref_dm(1,1)) << ", θ=" << double(ref_dm(2,1)) << std::endl;
    std::cerr << "[MPC调试] 参考轨迹 [N]: x=" << double(ref_dm(0,N)) << ", y=" << double(ref_dm(1,N)) << ", θ=" << double(ref_dm(2,N)) << std::endl;
    
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

    // Solve
    casadi::OptiSol sol = opti_.solve(); 
    
    solution.success = true;
    
    // Get Control
    DM u_opt = sol.value(U_);
    solution.cmd_vel.linear.x = double(u_opt(0,0));
    solution.cmd_vel.linear.y = double(u_opt(1,0));
    solution.cmd_vel.angular.z = double(u_opt(2,0));
    
    // Debug: 打印控制输出
    std::cerr << "[MPC调试] 控制输出: vx=" << solution.cmd_vel.linear.x 
              << ", vy=" << solution.cmd_vel.linear.y 
              << ", ω=" << solution.cmd_vel.angular.z << std::endl;
    
    // Check Slack
    // Check Slack
    DM eps_opt = sol.value(eps_slack_); // [K, N]
    double max_eps = 0.0;
    
    // 仅检查有效障碍物的松弛变量
    for(int j=0; j<n_obs; ++j) {
        for(int k=0; k<N; ++k) {
            double val = double(eps_opt(j,k));
            if(val > max_eps) max_eps = val;
        }
    }
    solution.max_slack = max_eps;
    
  } catch (const std::exception &e) {
    solution.success = false;
    std::cerr << "[MPC求解器] 求解失败: " << e.what() << std::endl;
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  solution.solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
  
  return solution;
}

}  // namespace dcs_nav_plugin
