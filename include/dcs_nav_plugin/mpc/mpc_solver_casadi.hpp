#ifndef DCS_NAV_PLUGIN__MPC__MPC_SOLVER_CASADI_HPP_
#define DCS_NAV_PLUGIN__MPC__MPC_SOLVER_CASADI_HPP_

#include <vector>
#include <memory>
#include <string>
#include <casadi/casadi.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "dcs_nav_plugin/geometry/geometry_engine.hpp"

namespace dcs_nav_plugin
{

struct MpcConfig {
  int N = 20;           // Prediction horizon steps
  double dt = 0.1;      // Time step (internal discretization)
  
  // Weights
  double Q_x = 10.0;
  double Q_y = 10.0;
  double Q_theta = 5.0;
  double R_v = 1.0;
  double R_omega = 1.0;
  double lambda_slack = 10000.0;
  
  // Constraints
  double v_max = 1.0;
  double omega_max = 1.0;
  double acc_max = 1.0;
  double margin = 0.1;
  double sht_epsilon = 1e-4; // For A^T A >= eps
  
  // Robot footprint (Box approx for SHT)
  double car_width = 0.5;
  double car_length = 0.8;
  double wheelbase = 0.5; // If using bicycle model
};

struct MpcSolution {
  bool success = false;
  geometry_msgs::msg::Twist cmd_vel;
  std::vector<geometry_msgs::msg::PoseStamped> predicted_traj;
  double solve_time_ms = 0.0;
  double max_slack = 0.0;
};

class MpcSolverCasadi
{
public:
  MpcSolverCasadi();
  ~MpcSolverCasadi();

  void configure(const MpcConfig & config);
  
  /**
   * @brief Initialize CasADi solver (symbolic graph construction)
   * This should be called once after configuration.
   */
  void initialize();

  /**
   * @brief Solve the MPC problem
   */
  MpcSolution solve(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_twist,
    const std::vector<geometry_msgs::msg::PoseStamped> & ref_traj,
    const std::vector<Polygon> & obstacles);

private:
  MpcConfig config_;
  bool initialized_ = false;

  // CasADi objects
  casadi::Opti opti_;
  casadi::Function solver_fn_; // Compiled solver function if we pre-generate
  
  // Re-use Opti instance method variables? 
  // CasADi Opti allows parametric optimization. 
  // We need to define Parameters for Initial State, Ref Trajectory, and Obstacles.
  
  // Since number of obstacles changes (Top-K), we might need fixed K slots.
  // Unused slots can be moved far away.
  int max_obstacles_ = 6; 
  int max_obs_vertices_ = 8; // Max vertices per obstacle polygon

  // Optimization Variables
  casadi::MX X_, U_;  // States, Controls
  casadi::MX A_sht_, b_sht_; // Hyperplanes [K, N, 2], [K, N]
  casadi::MX eps_slack_;     // Slack [K, N]

  // Parameters
  casadi::MX P_x0_;    // Initial state
  casadi::MX P_ref_;   // Reference trajectory [N+1, state_dim]
  
  // Obstacle Param Handles
  casadi::MX P_obs_Vx_;    // [K, max_verts]
  casadi::MX P_obs_Vy_;    // [K, max_verts]
  casadi::MX P_obs_valid_; // [K, 1]

  void buildProblem();
};

}  // namespace dcs_nav_plugin

#endif  // DCS_NAV_PLUGIN__MPC__MPC_SOLVER_CASADI_HPP_
