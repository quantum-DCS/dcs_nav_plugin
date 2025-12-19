#include "dcs_nav_plugin/hyperplane_planner.hpp"

#include <queue>
#include <limits>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "pluginlib/class_list_macros.hpp"

// CasADi
#include <casadi/casadi.hpp>

namespace dcs_nav_plugin
{

HyperplanePlanner::HyperplanePlanner() {}
HyperplanePlanner::~HyperplanePlanner() {}

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
  rclcpp::Parameter param;
  if (node->has_parameter(name_ + ".robot_radius")) {
    node->get_parameter(name_ + ".robot_radius", robot_radius_);
  } else {
    node->declare_parameter<double>(name_ + ".robot_radius", robot_radius_);
    node->get_parameter(name_ + ".robot_radius", robot_radius_);
  }
  node->declare_parameter<bool>(name_ + ".use_inflation_layer", use_inflation_layer_);
  node->get_parameter(name_ + ".use_inflation_layer", use_inflation_layer_);
  node->declare_parameter<int>(name_ + ".N", N_);
  node->get_parameter(name_ + ".N", N_);
  node->declare_parameter<int>(name_ + ".M", M_);
  node->get_parameter(name_ + ".M", M_);
  node->declare_parameter<double>(name_ + ".vmax_lin", vmax_lin_);
  node->get_parameter(name_ + ".vmax_lin", vmax_lin_);
  node->declare_parameter<double>(name_ + ".vmax_ang", vmax_ang_);
  node->get_parameter(name_ + ".vmax_ang", vmax_ang_);
  node->declare_parameter<double>(name_ + ".amax_lin", amax_lin_);
  node->get_parameter(name_ + ".amax_lin", amax_lin_);
  node->declare_parameter<double>(name_ + ".amax_ang", amax_ang_);
  node->get_parameter(name_ + ".amax_ang", amax_ang_);
  node->declare_parameter<double>(name_ + ".solver_time_limit_sec", solver_time_limit_sec_);
  node->get_parameter(name_ + ".solver_time_limit_sec", solver_time_limit_sec_);

  precomputeObstacles();

  RCLCPP_INFO(node->get_logger(), "HyperplanePlanner configured. Obstacles (convex pieces): %zu", convex_obstacles_.size());
}

void HyperplanePlanner::cleanup() {}
void HyperplanePlanner::activate() {}
void HyperplanePlanner::deactivate() {}

static inline cv::Point2f mapToWorld(nav2_costmap_2d::Costmap2D *cm, unsigned int mx, unsigned int my)
{
  double wx, wy;
  cm->mapToWorld(mx, my, wx, wy);
  return cv::Point2f(static_cast<float>(wx), static_cast<float>(wy));
}

void HyperplanePlanner::precomputeObstacles()
{
  convex_obstacles_.clear();
  obstacle_halfspaces_.clear();
  if (!costmap_) return;

  const unsigned int sx = costmap_->getSizeInCellsX();
  const unsigned int sy = costmap_->getSizeInCellsY();
  cv::Mat mask(static_cast<int>(sy), static_cast<int>(sx), CV_8UC1, cv::Scalar(0));
  const unsigned char * data = costmap_->getCharMap();
  for (unsigned int y = 0; y < sy; ++y) {
    for (unsigned int x = 0; x < sx; ++x) {
      unsigned char c = data[y * sx + x];
      bool occ = (c >= nav2_costmap_2d::LETHAL_OBSTACLE);
      if (use_inflation_layer_) {
        occ = occ || (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
      mask.at<uint8_t>(static_cast<int>(y), static_cast<int>(x)) = occ ? 255 : 0;
    }
  }

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point2f>> polys_world;
  extractContours(mask, polys_world);

  // Convex decomposition (initial version: convex hull of each polygon)
  std::vector<std::vector<cv::Point2f>> convex_parts;
  for (const auto &poly : polys_world) {
    std::vector<std::vector<cv::Point2f>> parts;
    convexDecomposeHM(poly, parts);
    for (auto &p : parts) convex_parts.push_back(std::move(p));
  }

  // Merge by robot radius
  mergeConvexByRadius(convex_parts, robot_radius_);
  convex_obstacles_ = convex_parts;

  // Precompute halfspaces
  for (const auto &conv : convex_obstacles_) {
    cv::Mat A, b;
    toHalfspaces(conv, A, b);
    obstacle_halfspaces_.emplace_back(A, b);
  }
}

void HyperplanePlanner::extractContours(cv::Mat &binary, std::vector<std::vector<cv::Point2f>> &polys_world)
{
  polys_world.clear();
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto &c : contours) {
    if (c.size() < 3) continue;
    std::vector<cv::Point> approx;
    cv::approxPolyDP(c, approx, 1.0, true);
    if (approx.size() < 3) continue;

    // Convert contour pixels to world coords
    std::vector<cv::Point2f> poly;
    poly.reserve(approx.size());
    for (const auto &pt : approx) {
      // contour point in image coords: (x=col, y=row)
      int cx = std::max(0, std::min(pt.x, static_cast<int>(costmap_->getSizeInCellsX()-1)));
      int cy = std::max(0, std::min(pt.y, static_cast<int>(costmap_->getSizeInCellsY()-1)));
      unsigned int mx = static_cast<unsigned int>(cx);
      unsigned int my = static_cast<unsigned int>(cy);
      poly.push_back(mapToWorld(costmap_, mx, my));
    }
    polys_world.push_back(std::move(poly));
  }
}

void HyperplanePlanner::convexDecomposeHM(const std::vector<cv::Point2f> &poly,
                                          std::vector<std::vector<cv::Point2f>> &convex_parts)
{
  // Initial simple version: use a single convex hull as conservative convex decomposition
  // (Hertel-Mehlhorn / ear clipping + greedy merge can be added later)
  if (poly.size() < 3) return;
  std::vector<cv::Point2f> hull;
  cv::convexHull(poly, hull, true, true);
  if (hull.size() >= 3) {
    convex_parts.push_back(std::move(hull));
  }
}

static inline float segDist(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &p)
{
  cv::Point2f ab = b - a;
  float t = 0.f;
  float denom = ab.dot(ab);
  if (denom > 1e-12f) {
    t = (p - a).dot(ab) / denom;
    t = std::min(1.f, std::max(0.f, t));
  }
  cv::Point2f proj = a + t * ab;
  return cv::norm(p - proj);
}

static float polyMinDistance(const std::vector<cv::Point2f> &A, const std::vector<cv::Point2f> &B)
{
  float dmin = std::numeric_limits<float>::infinity();
  // quick AABB rejection
  auto aabb = [](const std::vector<cv::Point2f> &P){
    float xmin=1e9, ymin=1e9, xmax=-1e9, ymax=-1e9;
    for (auto &p: P){ xmin = std::min(xmin, p.x); ymin = std::min(ymin, p.y); xmax = std::max(xmax, p.x); ymax = std::max(ymax, p.y);} 
    return cv::Rect2f(cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax));
  };
  cv::Rect2f ra = aabb(A), rb = aabb(B);
  if ((ra & rb).area() > 0) return 0.f; // overlapping AABB -> consider 0

  // precise min distance
  for (size_t i = 0; i < A.size(); ++i) {
    cv::Point2f a0 = A[i], a1 = A[(i+1)%A.size()];
    for (size_t j = 0; j < B.size(); ++j) {
      cv::Point2f b0 = B[j], b1 = B[(j+1)%B.size()];
      // segment-segment distance via sampling endpoints to opposite segment
      dmin = std::min(dmin, segDist(a0, a1, b0));
      dmin = std::min(dmin, segDist(a0, a1, b1));
      dmin = std::min(dmin, segDist(b0, b1, a0));
      dmin = std::min(dmin, segDist(b0, b1, a1));
    }
  }
  return dmin;
}

void HyperplanePlanner::mergeConvexByRadius(std::vector<std::vector<cv::Point2f>> &convex_parts, double radius)
{
  if (convex_parts.empty()) return;
  const float thresh = static_cast<float>(radius);

  bool merged = true;
  while (merged) {
    merged = false;
    for (size_t i = 0; i < convex_parts.size() && !merged; ++i) {
      for (size_t j = i + 1; j < convex_parts.size() && !merged; ++j) {
        float d = polyMinDistance(convex_parts[i], convex_parts[j]);
        if (d <= thresh) {
          std::vector<cv::Point2f> pts;
          pts.insert(pts.end(), convex_parts[i].begin(), convex_parts[i].end());
          pts.insert(pts.end(), convex_parts[j].begin(), convex_parts[j].end());
          std::vector<cv::Point2f> hull;
          cv::convexHull(pts, hull, true, true);
          convex_parts.erase(convex_parts.begin() + j);
          convex_parts.erase(convex_parts.begin() + i);
          convex_parts.insert(convex_parts.begin() + i, std::move(hull));
          merged = true;
        }
      }
    }
  }
}

void HyperplanePlanner::toHalfspaces(const std::vector<cv::Point2f> &convex, cv::Mat &A, cv::Mat &b)
{
  const int m = static_cast<int>(convex.size());
  A = cv::Mat::zeros(m, 2, CV_32F);
  b = cv::Mat::zeros(m, 1, CV_32F);
  for (int i = 0; i < m; ++i) {
    cv::Point2f p0 = convex[i];
    cv::Point2f p1 = convex[(i + 1) % m];
    cv::Point2f e = p1 - p0;           // edge vector
    cv::Point2f n(e.y, -e.x);          // outward normal for CCW polygon
    float norm = std::sqrt(n.dot(n));
    if (norm < 1e-6f) continue;
    n *= (1.0f / norm);
    A.at<float>(i, 0) = n.x;
    A.at<float>(i, 1) = n.y;
    b.at<float>(i, 0) = n.dot(p0);
  }
}

struct AStarNode { unsigned int x, y; float g, h; int parent_idx; };

static inline float heuristic(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1)
{ return std::hypot(static_cast<float>(x1) - x0, static_cast<float>(y1) - y0); }

bool HyperplanePlanner::runAStarPath(const geometry_msgs::msg::PoseStamped &start,
                                     const geometry_msgs::msg::PoseStamped &goal,
                                     std::vector<cv::Point2f> &path_world)
{
  path_world.clear();
  if (!costmap_) return false;

  unsigned int sx, sy, gx, gy;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) return false;
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) return false;

  const unsigned int nx = costmap_->getSizeInCellsX();
  const unsigned int ny = costmap_->getSizeInCellsY();
  const unsigned char * grid = costmap_->getCharMap();

  auto idx = [nx](unsigned int x, unsigned int y){ return y * nx + x; };
  auto freeCell = [&](unsigned int x, unsigned int y){
    unsigned char c = grid[idx(x,y)];
    return c < nav2_costmap_2d::LETHAL_OBSTACLE && c < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };

  std::vector<float> gscore(nx * ny, std::numeric_limits<float>::infinity());
  std::vector<int> parent(nx * ny, -1);
  std::vector<uint8_t> closed(nx * ny, 0);

  struct PQN { unsigned int x, y; float f; };
  struct Cmp { bool operator()(const PQN &a, const PQN &b) const { return a.f > b.f; } };
  std::priority_queue<PQN, std::vector<PQN>, Cmp> open;

  gscore[idx(sx,sy)] = 0.f;
  open.push({sx, sy, heuristic(sx,sy,gx,gy)});

  const int dx8[8] = {1,1,0,-1,-1,-1,0,1};
  const int dy8[8] = {0,1,1,1,0,-1,-1,-1};
  const float cost8[8] = {1, std::sqrt(2.f), 1, std::sqrt(2.f), 1, std::sqrt(2.f), 1, std::sqrt(2.f)};

  bool found = false;
  while (!open.empty()) {
    auto cur = open.top(); open.pop();
    unsigned int cx = cur.x, cy = cur.y;
    int cind = static_cast<int>(idx(cx,cy));
    if (closed[cind]) continue;
    closed[cind] = 1;
    if (cx == gx && cy == gy) { found = true; break; }

    for (int k = 0; k < 8; ++k) {
      int nx_ = static_cast<int>(cx) + dx8[k];
      int ny_ = static_cast<int>(cy) + dy8[k];
      if (nx_ < 0 || ny_ < 0 || nx_ >= static_cast<int>(nx) || ny_ >= static_cast<int>(ny)) continue;
      unsigned int ux = static_cast<unsigned int>(nx_);
      unsigned int uy = static_cast<unsigned int>(ny_);
      if (!freeCell(ux, uy)) continue;
      int uind = static_cast<int>(idx(ux,uy));
      float tentative = gscore[cind] + cost8[k];
      if (tentative < gscore[uind]) {
        gscore[uind] = tentative;
        parent[uind] = cind;
        float f = tentative + heuristic(ux,uy,gx,gy);
        open.push({ux, uy, f});
      }
    }
  }

  if (!found) return false;

  // reconstruct
  std::vector<cv::Point2f> rev;
  int cur = static_cast<int>(idx(gx,gy));
  while (cur >= 0) {
    unsigned int x = static_cast<unsigned int>(cur % nx);
    unsigned int y = static_cast<unsigned int>(cur / nx);
    rev.push_back(mapToWorld(costmap_, x, y));
    cur = parent[cur];
  }
  path_world.assign(rev.rbegin(), rev.rend());
  return true;
}

void HyperplanePlanner::interpolatePath(const std::vector<cv::Point2f> &path_world,
                                        size_t N,
                                        std::vector<cv::Point2f> &interp)
{
  interp.clear();
  if (path_world.empty()) return;
  if (path_world.size() >= N) {
    // downsample uniformly
    for (size_t i = 0; i < N; ++i) {
      size_t idx = static_cast<size_t>(std::round((static_cast<double>(i) / (N - 1)) * (path_world.size() - 1)));
      interp.push_back(path_world[idx]);
    }
    return;
  }
  // upsample by linear interpolation along cumulative length
  std::vector<double> s(path_world.size(), 0.0);
  for (size_t i = 1; i < path_world.size(); ++i) s[i] = s[i-1] + cv::norm(path_world[i] - path_world[i-1]);
  double total = s.back();
  interp.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    double si = (total * i) / (N - 1);
    // find segment
    size_t k = 1;
    while (k < s.size() && s[k] < si) ++k;
    if (k >= s.size()) { interp.push_back(path_world.back()); continue; }
    double t = (si - s[k-1]) / std::max(1e-6, s[k] - s[k-1]);
    cv::Point2f p = path_world[k-1] * static_cast<float>(1.0 - t) + path_world[k] * static_cast<float>(t);
    interp.push_back(p);
  }
}

nav_msgs::msg::Path HyperplanePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();

  // 1) Warmstart path by A*
  std::vector<cv::Point2f> warm;
  if (!runAStarPath(start, goal, warm) || warm.size() < 2) {
    RCLCPP_WARN(node->get_logger(), "HyperplanePlanner: A* warmstart failed or too short");
    return path; // empty indicates failure
  }

  // 2) Interpolate to N_+1 points
  std::vector<cv::Point2f> xref;
  interpolatePath(warm, static_cast<size_t>(N_ + 1), xref);

  // 3) Build and solve optimization (initial version: smooth positions with hyperplane separation)
  try {
    casadi::Opti opti;
    int T = N_ + 1;
    casadi::MX X = opti.variable(2, T); // positions only (x,y) for initial version

    // initialize with warmstart
    std::vector<double> Xinit(2 * T, 0.0);
    for (int k = 0; k < T; ++k) { Xinit[0 + 2*k] = xref[k].x; Xinit[1 + 2*k] = xref[k].y; }
    casadi::DM Xinit_mat = casadi::DM::reshape(casadi::DM(Xinit), 2, T);
    opti.set_initial(X, Xinit_mat);

    // boundary conditions
    opti.subject_to(X(0,0) == start.pose.position.x);
    opti.subject_to(X(1,0) == start.pose.position.y);
    opti.subject_to(X(0,T-1) == goal.pose.position.x);
    opti.subject_to(X(1,T-1) == goal.pose.position.y);

    // smoothing cost
    casadi::MX cost = casadi::MX::zeros(1);
    for (int k = 1; k < T-1; ++k) {
      casadi::MX d1 = X(casadi::Slice(), k) - X(casadi::Slice(), k-1);
      casadi::MX d2 = X(casadi::Slice(), k+1) - X(casadi::Slice(), k);
      casadi::MX diff = d2 - d1;
      cost += casadi::MX::dot(d1, d1) + casadi::MX::dot(d2, d2) + casadi::MX::dot(diff, diff);
    }

    // hyperplane separation per convex obstacle using a single hyperplane shared over all k (initial version)
    for (const auto &obs : convex_obstacles_) {
      // variables Aa (2x1), bb (1)
      casadi::MX Aa = opti.variable(2,1);
      casadi::MX bb = opti.variable(1,1);
      // norm constraint
      opti.subject_to(casadi::MX::dot(Aa, Aa) >= 1e-4);

      // vehicle footprint approximated as square of side 2*robot_radius_
      std::array<cv::Point2f,4> local = {
        cv::Point2f( robot_radius_,  robot_radius_),
        cv::Point2f( robot_radius_, -robot_radius_),
        cv::Point2f(-robot_radius_, -robot_radius_),
        cv::Point2f(-robot_radius_,  robot_radius_)
      };

      // ensure all time steps vehicle vertices are on one side
      for (int k = 0; k < T; ++k) {
        // fixed heading approximation from warm path
        float theta = 0.f;
        if (k+1 < T) theta = std::atan2(xref[k+1].y - xref[k].y, xref[k+1].x - xref[k].x);
        float c = std::cos(theta), s = std::sin(theta);
        for (int vi = 0; vi < 4; ++vi) {
          float vx = c*local[vi].x - s*local[vi].y + xref[k].x; // use warmstart for rotation and position
          float vy = s*local[vi].x + c*local[vi].y + xref[k].y;
          // Aa^T * [vx;vy] > bb
          opti.subject_to((casadi::MX) (Aa(0,0) * vx + Aa(1,0) * vy) >= bb + 1e-3);
        }
      }

      // obstacle vertices other side
      for (const auto &p : obs) {
        opti.subject_to((casadi::MX) (Aa(0,0) * p.x + Aa(1,0) * p.y) <= bb - 1e-3);
      }
    }

    opti.minimize(cost);
    casadi::Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = false;
    opts["ipopt.max_wall_time"] = solver_time_limit_sec_;
    opti.solver("ipopt", opts);
    auto sol = opti.solve();
    auto Xsol = sol.value(X);

    // output path
    path.poses.resize(T);
    for (int k = 0; k < T; ++k) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = static_cast<double>(Xsol(0,k));
      pose.pose.position.y = static_cast<double>(Xsol(1,k));
      pose.pose.position.z = 0.0;
      // orientation from warmstart tangent (optional)
      double th = 0.0;
      if (k+1 < T) th = std::atan2(xref[k+1].y - xref[k].y, xref[k+1].x - xref[k].x);
      pose.pose.orientation.z = std::sin(th*0.5);
      pose.pose.orientation.w = std::cos(th*0.5);
      path.poses[k] = std::move(pose);
    }
    return path;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "HyperplanePlanner optimization failed: %s", e.what());
    return nav_msgs::msg::Path();
  }
}

} // namespace dcs_nav_plugin

PLUGINLIB_EXPORT_CLASS(dcs_nav_plugin::HyperplanePlanner, nav2_core::GlobalPlanner)
