#include <gtest/gtest.h>
#include "dcs_nav_plugin/mpc/mpc_solver_casadi.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dcs_nav_plugin
{

class MpcSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    solver_ = std::make_shared<MpcSolverCasadi>();
    
    MpcConfig config;
    config.N = 10;  // 较短的时域便于测试
    config.dt = 0.1;
    config.v_max = 0.5;
    config.omega_max = 1.0;
    config.Q_x = 10.0;
    config.Q_y = 10.0;
    config.Q_theta = 5.0;
    config.R_v = 0.1;
    config.R_omega = 0.1;
    config.lambda_slack = 10000.0;
    config.margin = 0.1;
    config.sht_epsilon = 0.001;
    config.car_width = 0.3;
    config.car_length = 0.3;
    
    solver_->configure(config);
    solver_->initialize();
  }

  geometry_msgs::msg::PoseStamped createPose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    
    return pose;
  }

  std::shared_ptr<MpcSolverCasadi> solver_;
};

// 测试无障碍物时的基本路径跟踪
TEST_F(MpcSolverTest, BasicPathFollowing)
{
  // 当前位姿
  auto current_pose = createPose(0.0, 0.0, 0.0);
  geometry_msgs::msg::Twist current_twist;
  
  // 直线参考轨迹
  std::vector<geometry_msgs::msg::PoseStamped> ref_traj;
  for (int i = 0; i <= 10; ++i) {
    ref_traj.push_back(createPose(i * 0.1, 0.0, 0.0));
  }
  
  // 无障碍物
  std::vector<Polygon> obstacles;
  
  // 求解
  auto solution = solver_->solve(current_pose, current_twist, ref_traj, obstacles);
  
  // 验证求解成功
  EXPECT_TRUE(solution.success);
  
  // 验证速度方向正确（前进）
  EXPECT_GT(solution.cmd_vel.linear.x, 0.0);
  
  // 验证求解时间合理（< 500ms）
  EXPECT_LT(solution.solve_time_ms, 500.0);
}

// 测试单个障碍物存在时的求解
TEST_F(MpcSolverTest, SingleObstaclePresent)
{
  auto current_pose = createPose(0.0, 0.0, 0.0);
  geometry_msgs::msg::Twist current_twist;
  
  std::vector<geometry_msgs::msg::PoseStamped> ref_traj;
  for (int i = 0; i <= 10; ++i) {
    ref_traj.push_back(createPose(i * 0.1, 0.0, 0.0));
  }
  
  // 在路径前方放置障碍物
  Polygon obstacle;
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = 0.5; p1.y = -0.3;
  p2.x = 0.8; p2.y = -0.3;
  p3.x = 0.8; p3.y = 0.3;
  p4.x = 0.5; p4.y = 0.3;
  obstacle.push_back(p1);
  obstacle.push_back(p2);
  obstacle.push_back(p3);
  obstacle.push_back(p4);
  
  std::vector<Polygon> obstacles = {obstacle};
  
  // 求解（可能成功或失败，取决于 MPC 能否找到可行解）
  auto solution = solver_->solve(current_pose, current_twist, ref_traj, obstacles);
  
  // 验证求解时间合理
  EXPECT_LT(solution.solve_time_ms, 1000.0);
}

// 测试 Warm Start 效果
TEST_F(MpcSolverTest, WarmStartInitialization)
{
  auto current_pose = createPose(0.0, 0.0, 0.0);
  geometry_msgs::msg::Twist current_twist;
  
  std::vector<geometry_msgs::msg::PoseStamped> ref_traj;
  for (int i = 0; i <= 10; ++i) {
    ref_traj.push_back(createPose(i * 0.05, 0.0, 0.0));
  }
  
  std::vector<Polygon> obstacles;
  
  // 第一次求解
  auto sol1 = solver_->solve(current_pose, current_twist, ref_traj, obstacles);
  
  // 稍微移动后再次求解
  auto next_pose = createPose(0.05, 0.0, 0.0);
  auto sol2 = solver_->solve(next_pose, current_twist, ref_traj, obstacles);
  
  // 验证两次都成功
  EXPECT_TRUE(sol1.success);
  EXPECT_TRUE(sol2.success);
}

}  // namespace dcs_nav_plugin

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
