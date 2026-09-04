/**
 * @file trajectory_test_node.cpp
 * @brief Safe, feasible trajectory generator for testing the geometric controller.
 *
 * Publishes mav_controllers_ros/TargetCommand (position, velocity, acceleration,
 * jerk, yaw, yaw_dot) on geometric_controller/setpoint.
 *
 * Trajectory types:
 *   - setpoint:   minimum-jerk point-to-point move, then hold.
 *   - circle:     circular orbit, entered at the nearest point on the circle.
 *   - lemniscate: Gerono figure-8, entered at the nearest point on the curve.
 *
 * Safety design:
 *   - Always streams a hold setpoint at the current pose while idle, so PX4
 *     OFFBOARD can be engaged safely (setpoint stream exists before engage).
 *   - Tracking only runs while OFFBOARD + motors enabled + odometry fresh;
 *     any loss of these reverts to HOLD and requires a new start trigger.
 *   - The whole planned path is checked against a geofence before starting;
 *     a runtime geofence breach aborts to HOLD.
 *   - Periodic trajectories are speed/accel/jerk limited: the angular rate is
 *     derated in closed form from the curve's max |p'|, |p''|, |p'''| so the
 *     commanded reference never exceeds the configured limits.
 *   - Speed ramps in/out with a C2 smoothstep profile, so velocity,
 *     acceleration and jerk feedforward stay consistent and continuous.
 *   - Yaw is rate limited with matching yaw_dot feedforward.
 *
 * Services (std_srvs/Trigger):
 *   ~/start : plan from the current state and begin the trajectory.
 *   ~/stop  : ramp down smoothly and hold.
 *
 * Status:
 *   trajectory_test/status : std_msgs/String, the phase name, published on
 *     every transition.
 *   trajectory_test/health : diagnostic_msgs/DiagnosticStatus at a steady
 *     rate. The String above only fires on a transition, so a ground station
 *     that connects mid-flight sees nothing and cannot tell WHY a start would
 *     be refused. This carries the phase, each engagement precondition, the
 *     achieved-vs-requested speed after derating, geofence margins, and the
 *     last plan report.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "mav_controllers_ros/msg/target_command.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

constexpr double kMinGotoTime = 1.0;       // [s] lower bound on point-to-point time
constexpr int kFeasibilityGridN = 2000;    // samples over one period for curve scan

double wrapPi(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// C2 smoothstep and derivatives: h(0)=0, h(1)=1, h'(0)=h'(1)=h''(0)=h''(1)=0
double smoothstep(double t)   { return t * t * t * (10.0 + t * (-15.0 + 6.0 * t)); }
double smoothstepD1(double t) { return t * t * (30.0 + t * (-60.0 + 30.0 * t)); }
double smoothstepD2(double t) { return t * (60.0 + t * (-180.0 + 120.0 * t)); }
double smoothstepD3(double t) { return 60.0 + t * (-360.0 + 360.0 * t); }
// Integral of smoothstep, H(0)=0, H(1)=0.5
double smoothstepInt(double t)
{
  const double t4 = t * t * t * t;
  return t4 * (2.5 + t * (-3.0 + t));
}

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
  Vec3 operator+(const Vec3 & o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 & o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

// Full reference state handed to the controller.
struct RefState
{
  Vec3 p, v, a, j;
  double yaw{0.0};
  double yaw_dot{0.0};
};

}  // namespace

class TrajectoryTestNode : public rclcpp::Node
{
public:
  TrajectoryTestNode() : Node("trajectory_test_node")
  {
    declareParameters();

    setpoint_pub_ = create_publisher<mav_controllers_ros::msg::TargetCommand>(
        "geometric_controller/setpoint", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "trajectory_test/planned_path", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("trajectory_test/status", 10);
    // Steady-rate health for a ground station: the String above only fires
    // on a phase transition, so a panel that connects mid-flight would see
    // nothing at all until something changed.
    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "trajectory_test/health", 10);

    // RViz tracking-quality visualization: the commanded reference trace, the
    // actually flown trace (overlay them to see tracking error), and the
    // current setpoint pose (arrow shows commanded yaw).
    ref_path_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory_test/reference_path", 10);
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory_test/actual_path", 10);
    setpoint_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("trajectory_test/setpoint_pose", 10);
    ref_path_msg_.header.frame_id = "map";
    actual_path_msg_.header.frame_id = "map";

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "mavros/local_position/odom", rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryTestNode::odomCallback, this, _1));
    mavros_state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&TrajectoryTestNode::mavrosStateCallback, this, _1));
    enable_motors_sub_ = create_subscription<std_msgs::msg::Bool>(
        "geometric_controller/enable_motors", 10,
        std::bind(&TrajectoryTestNode::motorsCallback, this, _1));

    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "trajectory_test/start", std::bind(&TrajectoryTestNode::startService, this, _1, _2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "trajectory_test/stop", std::bind(&TrajectoryTestNode::stopService, this, _1, _2));

    declare_parameter("health_rate", 5.0);
    const double health_rate = get_parameter("health_rate").as_double();
    if (health_rate > 0.0) {
      health_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / health_rate),
                                        std::bind(&TrajectoryTestNode::publishHealth, this));
    }

    const double rate = get_parameter("publish_rate").as_double();
    dt_ = 1.0 / std::max(rate, 1.0);
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                               std::bind(&TrajectoryTestNode::update, this));

    RCLCPP_INFO(get_logger(),
                "trajectory_test_node up: type='%s', auto_start=%s. "
                "Streaming hold setpoints; call ~/start (Trigger) to fly the trajectory.",
                get_parameter("trajectory_type").as_string().c_str(),
                get_parameter("auto_start").as_bool() ? "true" : "false");
  }

private:
  // ---------------------------------------------------------------- phases
  enum class Phase { HOLD, GOTO, TRACK, STOPPING };

  // ------------------------------------------------------------ parameters
  void declareParameters()
  {
    declare_parameter("trajectory_type", std::string("circle"));  // setpoint|circle|lemniscate
    declare_parameter("publish_rate", 100.0);                     // [Hz]

    // Interpret setpoint/center as offsets from the position at start trigger
    // (true, safest in the field) or as absolute local-frame coordinates.
    declare_parameter("relative_to_start", true);

    declare_parameter("setpoint.x", 0.0);
    declare_parameter("setpoint.y", 0.0);
    declare_parameter("setpoint.z", 0.0);
    declare_parameter("setpoint.yaw", 0.0);   // absolute [rad]; used when yaw_mode==fixed

    declare_parameter("circle.center_x", 0.0);
    declare_parameter("circle.center_y", 0.0);
    declare_parameter("circle.z", 0.0);
    declare_parameter("circle.radius", 3.0);
    declare_parameter("circle.clockwise", false);

    declare_parameter("lemniscate.center_x", 0.0);
    declare_parameter("lemniscate.center_y", 0.0);
    declare_parameter("lemniscate.z", 0.0);
    declare_parameter("lemniscate.width", 4.0);   // half-length A: x spans [-A, A]

    declare_parameter("speed", 2.0);              // [m/s] desired path speed
    declare_parameter("ramp_time", 4.0);          // [s] speed ramp in/out duration

    declare_parameter("goto_speed", 1.0);         // [m/s] peak speed of transition moves
    declare_parameter("goto_accel", 1.0);         // [m/s^2] peak accel of transition moves

    declare_parameter("limits.max_speed", 8.0);       // [m/s]
    declare_parameter("limits.max_accel", 5.0);       // [m/s^2] (keep < controller max_accel)
    declare_parameter("limits.max_jerk", 40.0);       // [m/s^3]
    declare_parameter("limits.max_yaw_rate", 1.0);    // [rad/s]

    declare_parameter("yaw_mode", std::string("tangent"));  // hold|fixed|tangent|center

    declare_parameter("geofence.enable", true);
    declare_parameter("geofence.max_radius_xy", 20.0);  // [m] from position at start trigger
    declare_parameter("geofence.min_z", 0.5);           // [m] local frame
    declare_parameter("geofence.max_z", 30.0);          // [m] local frame

    declare_parameter("auto_start", false);   // start as soon as engaged (SITL convenience)
    declare_parameter("odom_timeout", 0.5);   // [s]

    declare_parameter("viz.path_max_points", 4000);   // rolling window per path
    declare_parameter("viz.min_point_dist", 0.05);    // [m] decimation between points
  }

  // ------------------------------------------------------------- callbacks
  void odomCallback(const nav_msgs::msg::Odometry & msg)
  {
    odom_pos_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;
    last_odom_time_ = now();
    have_odom_ = true;
    appendToPath(actual_path_msg_, odom_pos_, msg.pose.pose.orientation);
  }

  static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
  }

  void appendToPath(nav_msgs::msg::Path & path, const Vec3 & p,
                    const geometry_msgs::msg::Quaternion & q)
  {
    const double min_d = get_parameter("viz.min_point_dist").as_double();
    if (!path.poses.empty()) {
      const auto & last = path.poses.back().pose.position;
      const Vec3 d{p.x - last.x, p.y - last.y, p.z - last.z};
      if (d.norm() < min_d) return;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = path.header.frame_id;
    ps.header.stamp = now();
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = p.z;
    ps.pose.orientation = q;
    path.poses.push_back(ps);
    const auto max_pts = static_cast<size_t>(get_parameter("viz.path_max_points").as_int());
    if (path.poses.size() > max_pts)
      path.poses.erase(path.poses.begin(),
                       path.poses.begin() + (path.poses.size() - max_pts));
  }

  void mavrosStateCallback(const mavros_msgs::msg::State & msg)
  {
    offboard_ = (msg.mode == mavros_msgs::msg::State::MODE_PX4_OFFBOARD);
    armed_ = msg.armed;
  }

  void motorsCallback(const std_msgs::msg::Bool & msg) { motors_enabled_ = msg.data; }

  void startService(const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::string why;
    if (!engaged(why)) {
      res->success = false;
      res->message = "Cannot start: " + why;
      return;
    }
    if (phase_ != Phase::HOLD) {
      res->success = false;
      res->message = "Cannot start: trajectory already active (call ~/stop first).";
      return;
    }
    std::string report;
    if (!planFromCurrentState(report)) {
      last_plan_report_ = report;
      last_plan_ok_ = false;
      res->success = false;
      res->message = "Plan rejected: " + report;
      return;
    }
    start_requested_ = true;
    res->success = true;
    res->message = report;
  }

  void stopService(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    stop_requested_ = true;
    start_requested_ = false;
    res->success = true;
    res->message = "Stop requested: ramping down to hold.";
  }

  // ----------------------------------------------------------- engagement
  bool odomFresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() <
           get_parameter("odom_timeout").as_double();
  }

  bool engaged(std::string & why) const
  {
    if (!odomFresh()) { why = "odometry missing or stale"; return false; }
    if (!offboard_)   { why = "not in OFFBOARD mode"; return false; }
    if (!armed_)      { why = "vehicle not armed"; return false; }
    if (!motors_enabled_) { why = "controller motors not enabled"; return false; }
    return true;
  }

  // ----------------------------------------------------- parametric curves
  // Curve position and derivatives w.r.t. the parameter theta (not time).
  // type_: 1 = circle, 2 = lemniscate (Gerono).
  void curve(double theta, Vec3 & p, Vec3 & d1, Vec3 & d2, Vec3 & d3) const
  {
    if (traj_type_ == 1) {
      const double c = std::cos(theta), s = std::sin(theta);
      const double r = radius_;
      p  = {center_.x + r * c, center_.y + r * s, center_.z};
      d1 = {-r * s,  r * c, 0.0};
      d2 = {-r * c, -r * s, 0.0};
      d3 = { r * s, -r * c, 0.0};
    } else {  // Gerono lemniscate: x = A sin(th), y = (A/2) sin(2 th)
      const double A = width_;
      const double s = std::sin(theta), c = std::cos(theta);
      const double s2 = std::sin(2.0 * theta), c2 = std::cos(2.0 * theta);
      p  = {center_.x + A * s, center_.y + 0.5 * A * s2, center_.z};
      d1 = { A * c,        A * c2,        0.0};
      d2 = {-A * s, -2.0 * A * s2,        0.0};
      d3 = {-A * c, -4.0 * A * c2,        0.0};
    }
  }

  // Reference state on the curve given theta and its time derivatives.
  RefState curveRef(double theta, double th_d, double th_dd, double th_ddd) const
  {
    Vec3 p, d1, d2, d3;
    curve(theta, p, d1, d2, d3);
    RefState r;
    r.p = p;
    r.v = d1 * th_d;
    r.a = d2 * (th_d * th_d) + d1 * th_dd;
    r.j = d3 * (th_d * th_d * th_d) + d2 * (3.0 * th_d * th_dd) + d1 * th_ddd;

    // Yaw reference and analytic yaw rate.
    if (yaw_mode_ == YawMode::TANGENT) {
      // Direction of travel: sign of nominal theta rate.
      const double sgn = (dir_ >= 0) ? 1.0 : -1.0;
      r.yaw = std::atan2(sgn * d1.y, sgn * d1.x);
      const double den = d1.x * d1.x + d1.y * d1.y;
      if (den > 1e-9)
        r.yaw_dot = (d1.x * d2.y - d1.y * d2.x) / den * th_d;
    } else if (yaw_mode_ == YawMode::CENTER) {
      const double dx = center_.x - p.x, dy = center_.y - p.y;
      const double den = dx * dx + dy * dy;
      r.yaw = std::atan2(dy, dx);
      if (den > 1e-9)
        r.yaw_dot = (r.v.x * dy - r.v.y * dx) / den;
    } else {
      r.yaw = fixed_yaw_;
      r.yaw_dot = 0.0;
    }
    return r;
  }

  // ----------------------------------------------------------- planning
  // Reads params, captures the current state as the plan origin, runs the
  // feasibility scan and geofence pre-check. Returns false with an
  // explanation if the plan is unsafe.
  bool planFromCurrentState(std::string & report)
  {
    const std::string type = get_parameter("trajectory_type").as_string();
    if (type == "setpoint") traj_type_ = 0;
    else if (type == "circle") traj_type_ = 1;
    else if (type == "lemniscate") traj_type_ = 2;
    else { report = "unknown trajectory_type '" + type + "'"; return false; }

    const std::string ymode = get_parameter("yaw_mode").as_string();
    if (ymode == "hold") yaw_mode_ = YawMode::HOLD;
    else if (ymode == "fixed") yaw_mode_ = YawMode::FIXED;
    else if (ymode == "tangent") yaw_mode_ = YawMode::TANGENT;
    else if (ymode == "center") yaw_mode_ = YawMode::CENTER;
    else { report = "unknown yaw_mode '" + ymode + "'"; return false; }

    origin_ = odom_pos_;
    origin_yaw_ = odom_yaw_;
    const bool rel = get_parameter("relative_to_start").as_bool();
    const Vec3 off = rel ? origin_ : Vec3{0.0, 0.0, 0.0};

    v_max_ = get_parameter("limits.max_speed").as_double();
    a_max_ = get_parameter("limits.max_accel").as_double();
    j_max_ = get_parameter("limits.max_jerk").as_double();
    yaw_rate_max_ = get_parameter("limits.max_yaw_rate").as_double();
    ramp_time_ = std::max(get_parameter("ramp_time").as_double(), 0.5);
    fixed_yaw_ = (yaw_mode_ == YawMode::HOLD)
                     ? origin_yaw_
                     : get_parameter("setpoint.yaw").as_double();

    char buf[512];

    if (traj_type_ == 0) {
      goto_target_.p = Vec3{get_parameter("setpoint.x").as_double(),
                            get_parameter("setpoint.y").as_double(),
                            get_parameter("setpoint.z").as_double()} + off;
      goto_target_.yaw = (yaw_mode_ == YawMode::FIXED) ? get_parameter("setpoint.yaw").as_double()
                                                       : origin_yaw_;
      snprintf(buf, sizeof(buf), "setpoint move to (%.1f, %.1f, %.1f), min-jerk.",
               goto_target_.p.x, goto_target_.p.y, goto_target_.p.z);
    } else {
      if (traj_type_ == 1) {
        radius_ = get_parameter("circle.radius").as_double();
        if (radius_ < 0.5) { report = "circle.radius must be >= 0.5 m"; return false; }
        center_ = Vec3{get_parameter("circle.center_x").as_double(),
                       get_parameter("circle.center_y").as_double(),
                       get_parameter("circle.z").as_double()} + off;
        dir_ = get_parameter("circle.clockwise").as_bool() ? -1.0 : 1.0;
      } else {
        width_ = get_parameter("lemniscate.width").as_double();
        if (width_ < 1.0) { report = "lemniscate.width must be >= 1.0 m"; return false; }
        center_ = Vec3{get_parameter("lemniscate.center_x").as_double(),
                       get_parameter("lemniscate.center_y").as_double(),
                       get_parameter("lemniscate.z").as_double()} + off;
        dir_ = 1.0;
      }

      // Feasibility: scan the curve for max parametric derivatives, then the
      // admissible constant angular rate is
      //   omega = min( v/|p'|max, sqrt(a_max/|p''|max), cbrt(j_max/|p'''|max) )
      // since speed ~ omega, accel ~ omega^2, jerk ~ omega^3 at constant rate.
      double m1 = 0.0, m2 = 0.0, m3 = 0.0;
      double th_nearest = 0.0, best_d = 1e18;
      for (int i = 0; i < kFeasibilityGridN; ++i) {
        const double th = 2.0 * M_PI * i / static_cast<double>(kFeasibilityGridN);
        Vec3 p, d1, d2, d3;
        curve(th, p, d1, d2, d3);
        m1 = std::max(m1, d1.norm());
        m2 = std::max(m2, d2.norm());
        m3 = std::max(m3, d3.norm());
        const double d = (p - origin_).norm();
        if (d < best_d) { best_d = d; th_nearest = th; }
      }
      const double v_des = std::min(get_parameter("speed").as_double(), v_max_);
      const double w_v = v_des / m1;
      const double w_a = std::sqrt(a_max_ / m2);
      const double w_j = std::cbrt(j_max_ / m3);
      omega_ = std::min({w_v, w_a, w_j});
      // Ramp adds a p'*theta_dd term; verify the ramp keeps accel margin.
      const double ramp_acc_extra = m1 * omega_ * smoothstepD1(0.5) / ramp_time_;
      if (m2 * omega_ * omega_ + ramp_acc_extra > a_max_)
        omega_ = std::min(omega_, std::sqrt(std::max(a_max_ - ramp_acc_extra, 0.1) / m2));

      const double v_ach = omega_ * m1, a_ach = omega_ * omega_ * m2;
      v_achieved_ = v_ach;
      a_achieved_ = a_ach;
      entry_distance_ = best_d;
      theta0_ = th_nearest;
      Vec3 p0, d1, d2, d3;
      curve(theta0_, p0, d1, d2, d3);
      goto_target_.p = p0;
      // Face the initial direction of travel (or per yaw mode) before starting.
      goto_target_.yaw = curveRef(theta0_, 0.0, 0.0, 0.0).yaw;

      snprintf(buf, sizeof(buf),
               "%s: peak speed %.2f m/s (requested %.2f), peak accel %.2f m/s^2, "
               "omega %.3f rad/s, entering at nearest point %.1f m away.",
               type.c_str(), v_ach, v_des, a_ach, omega_, best_d);
      if (v_ach < v_des - 0.05)
        RCLCPP_WARN(get_logger(),
                    "Requested speed %.2f m/s derated to %.2f m/s by accel/jerk limits "
                    "(a_max=%.1f, j_max=%.1f). Increase radius/width for more speed.",
                    v_des, v_ach, a_max_, j_max_);
    }

    // --- geofence pre-check of the entire planned path
    if (get_parameter("geofence.enable").as_bool()) {
      const double rmax = get_parameter("geofence.max_radius_xy").as_double();
      const double zmin = get_parameter("geofence.min_z").as_double();
      const double zmax = get_parameter("geofence.max_z").as_double();
      // Name the limit that was breached and by how much. "violates the
      // geofence" sends you reading yaml to work out which of three
      // numbers you hit -- and the two that are easy to confuse are the
      // XY radius, measured from the pose at START, and the z limits,
      // which are absolute altitudes in the local frame.
      char fence_buf[256] = {0};
      auto inFence = [&](const Vec3 & p) {
        const double dxy = std::hypot(p.x - origin_.x, p.y - origin_.y);
        if (dxy > rmax) {
          snprintf(fence_buf, sizeof(fence_buf),
                   "path point (%.1f, %.1f, %.1f) is %.1f m from the start pose, "
                   "beyond geofence.max_radius_xy = %.1f m",
                   p.x, p.y, p.z, dxy, rmax);
          return false;
        }
        if (p.z < zmin) {
          snprintf(fence_buf, sizeof(fence_buf),
                   "path point (%.1f, %.1f, %.1f) is below geofence.min_z = %.1f m. "
                   "With relative_to_start disabled, %s is an ABSOLUTE local-frame "
                   "altitude measured from the frame origin, not from the vehicle, so "
                   "0 is the ground.",
                   p.x, p.y, p.z, zmin,
                   traj_type_ == 0 ? "setpoint.z"
                                   : (traj_type_ == 1 ? "circle.z" : "lemniscate.z"));
          return false;
        }
        if (p.z > zmax) {
          snprintf(fence_buf, sizeof(fence_buf),
                   "path point (%.1f, %.1f, %.1f) is above geofence.max_z = %.1f m",
                   p.x, p.y, p.z, zmax);
          return false;
        }
        return true;
      };
      bool ok = inFence(goto_target_.p);
      if (traj_type_ != 0) {
        for (int i = 0; ok && i < 400; ++i) {
          Vec3 p, d1, d2, d3;
          curve(2.0 * M_PI * i / 400.0, p, d1, d2, d3);
          ok = inFence(p);
        }
      }
      if (!ok) {
        report = std::string("geofence: ") + fence_buf;
        return false;
      }
    }

    // Fresh traces for the new run so RViz shows only this test.
    ref_path_msg_.poses.clear();
    actual_path_msg_.poses.clear();

    publishPlannedPath();
    report = buf;
    last_plan_report_ = report;
    last_plan_ok_ = true;
    RCLCPP_INFO(get_logger(), "Plan accepted: %s", report.c_str());
    return true;
  }

  void publishPlannedPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    auto addPoint = [&](const Vec3 & p) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = p.z;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    };
    addPoint(origin_);
    addPoint(goto_target_.p);
    if (traj_type_ != 0) {
      for (int i = 0; i <= 200; ++i) {
        Vec3 p, d1, d2, d3;
        curve(theta0_ + dir_ * 2.0 * M_PI * i / 200.0, p, d1, d2, d3);
        addPoint(p);
      }
    }
    path_pub_->publish(path);
  }

  // Minimum-jerk point-to-point segment setup (position + yaw blend).
  void startGoto(const Vec3 & from, double from_yaw)
  {
    goto_start_.p = from;
    goto_start_.yaw = from_yaw;
    const double D = (goto_target_.p - from).norm();
    const double dyaw = std::fabs(wrapPi(goto_target_.yaw - from_yaw));
    const double v = std::max(get_parameter("goto_speed").as_double(), 0.1);
    const double a = std::max(get_parameter("goto_accel").as_double(), 0.1);
    // Min-jerk peaks: v_pk = 1.875 D/T, a_pk = 5.774 D/T^2, j_pk = 60 D/T^3.
    goto_T_ = std::max({kMinGotoTime,
                        1.875 * D / v,
                        std::sqrt(5.7735 * D / a),
                        std::cbrt(60.0 * D / j_max_),
                        dyaw / std::max(0.5 * yaw_rate_max_, 0.05)});
    phase_time_ = 0.0;
    setPhase(Phase::GOTO);
  }

  RefState gotoRef(double t) const
  {
    const double tau = std::clamp(t / goto_T_, 0.0, 1.0);
    const Vec3 D = goto_target_.p - goto_start_.p;
    const double dyaw = wrapPi(goto_target_.yaw - goto_start_.yaw);
    RefState r;
    r.p = goto_start_.p + D * smoothstep(tau);
    r.v = D * (smoothstepD1(tau) / goto_T_);
    r.a = D * (smoothstepD2(tau) / (goto_T_ * goto_T_));
    r.j = D * (smoothstepD3(tau) / (goto_T_ * goto_T_ * goto_T_));
    r.yaw = goto_start_.yaw + dyaw * smoothstep(tau);
    r.yaw_dot = dyaw * smoothstepD1(tau) / goto_T_;
    return r;
  }

  // Theta profile during TRACK: smoothstep ramp to omega_, then constant.
  void trackTheta(double t, double & th, double & th_d, double & th_dd, double & th_ddd) const
  {
    const double T = ramp_time_;
    if (t < T) {
      const double tau = t / T;
      th     = dir_ * omega_ * T * smoothstepInt(tau);
      th_d   = dir_ * omega_ * smoothstep(tau);
      th_dd  = dir_ * omega_ * smoothstepD1(tau) / T;
      th_ddd = dir_ * omega_ * smoothstepD2(tau) / (T * T);
    } else {
      th     = dir_ * omega_ * (t - 0.5 * T);
      th_d   = dir_ * omega_;
      th_dd  = 0.0;
      th_ddd = 0.0;
    }
  }

  // Theta profile during STOPPING: ramp from omega_ to zero.
  void stopTheta(double t, double & th, double & th_d, double & th_dd, double & th_ddd) const
  {
    const double T = ramp_time_;
    const double tau = std::clamp(t / T, 0.0, 1.0);
    th     = stop_theta0_ + dir_ * omega_ * T * (tau - smoothstepInt(tau));
    th_d   = dir_ * omega_ * (1.0 - smoothstep(tau));
    th_dd  = -dir_ * omega_ * smoothstepD1(tau) / T;
    th_ddd = -dir_ * omega_ * smoothstepD2(tau) / (T * T);
  }

  // -------------------------------------------------------------- main loop
  void update()
  {
    // No odometry: publish nothing, the controller's own watchdogs take over.
    if (!odomFresh()) {
      if (phase_ != Phase::HOLD) {
        RCLCPP_ERROR(get_logger(), "Odometry lost: aborting trajectory, not publishing.");
        start_requested_ = false;
        setPhase(Phase::HOLD);
        hold_valid_ = false;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for odometry on mavros/local_position/odom ...");
      return;
    }

    // Disengage (mode change / disarm / motors off) aborts any active phase.
    std::string why;
    const bool is_engaged = engaged(why);
    if (!is_engaged && phase_ != Phase::HOLD) {
      RCLCPP_WARN(get_logger(), "Disengaged (%s): reverting to HOLD at current pose.", why.c_str());
      start_requested_ = false;
      setPhase(Phase::HOLD);
      hold_valid_ = false;
    }

    // While not engaged, keep the hold point glued to the vehicle so the
    // reference is sane the moment OFFBOARD engages.
    if (!is_engaged) hold_valid_ = false;

    if (!hold_valid_) {
      hold_ref_ = RefState{};
      hold_ref_.p = odom_pos_;
      hold_ref_.yaw = odom_yaw_;
      hold_valid_ = true;
    }

    // Runtime geofence on actual vehicle position (only while trajectory active).
    if (phase_ != Phase::HOLD && get_parameter("geofence.enable").as_bool()) {
      const double rmax = get_parameter("geofence.max_radius_xy").as_double();
      const double zmin = get_parameter("geofence.min_z").as_double();
      const double zmax = get_parameter("geofence.max_z").as_double();
      const double dxy = std::hypot(odom_pos_.x - origin_.x, odom_pos_.y - origin_.y);
      if (dxy > rmax || odom_pos_.z < zmin || odom_pos_.z > zmax) {
        RCLCPP_ERROR(get_logger(),
                     "GEOFENCE BREACH (dxy=%.1f m, z=%.1f m): holding at current pose.",
                     dxy, odom_pos_.z);
        start_requested_ = false;
        setPhase(Phase::HOLD);
        hold_ref_ = RefState{};
        hold_ref_.p = odom_pos_;
        hold_ref_.yaw = odom_yaw_;
        hold_valid_ = true;
      }
    }

    // Auto-start (SITL convenience): trigger once when first engaged.
    if (is_engaged && phase_ == Phase::HOLD && !start_requested_ &&
        get_parameter("auto_start").as_bool() && !auto_start_done_) {
      std::string report;
      if (planFromCurrentState(report)) {
        start_requested_ = true;
        auto_start_done_ = true;
      } else {
        RCLCPP_ERROR(get_logger(), "auto_start plan rejected: %s", report.c_str());
        auto_start_done_ = true;  // don't spam
      }
    }

    phase_time_ += dt_;

    RefState ref;
    switch (phase_) {
      case Phase::HOLD:
        ref = hold_ref_;
        if (is_engaged && start_requested_) {
          start_requested_ = false;
          stop_requested_ = false;
          startGoto(hold_ref_.p, hold_ref_.yaw);
          ref = gotoRef(0.0);
        }
        break;

      case Phase::GOTO:
        ref = gotoRef(phase_time_);
        if (stop_requested_) {
          // Finish the (already smooth, bounded) move, then hold there.
          stop_requested_ = false;
          traj_type_after_goto_hold_ = true;
        }
        if (phase_time_ >= goto_T_) {
          if (traj_type_ == 0 || traj_type_after_goto_hold_) {
            hold_ref_ = RefState{};
            hold_ref_.p = goto_target_.p;
            hold_ref_.yaw = goto_target_.yaw;
            hold_valid_ = true;
            traj_type_after_goto_hold_ = false;
            setPhase(Phase::HOLD);
            ref = hold_ref_;
          } else {
            setPhase(Phase::TRACK);
            phase_time_ = 0.0;
            ref = trackRef(0.0);
          }
        }
        break;

      case Phase::TRACK: {
        ref = trackRef(phase_time_);
        if (stop_requested_) {
          stop_requested_ = false;
          double th, th_d, th_dd, th_ddd;
          trackTheta(phase_time_, th, th_d, th_dd, th_ddd);
          // Enter the ramp-down at the point of matching angular rate so the
          // reference velocity is continuous. h() is symmetric about 0.5, so
          // if we are still ramping up at tau_u, the matching down-ramp time
          // is tau_s = 1 - tau_u; at cruise the down-ramp starts at tau_s = 0.
          double tau_s = 0.0;
          if (phase_time_ < ramp_time_)
            tau_s = 1.0 - phase_time_ / ramp_time_;
          // Choose stop_theta0_ so the down-ramp position is continuous.
          const double t_s = tau_s * ramp_time_;
          stop_theta0_ = (theta0_ + th) -
                         dir_ * omega_ * ramp_time_ * (tau_s - smoothstepInt(tau_s));
          setPhase(Phase::STOPPING);
          phase_time_ = t_s;
          ref = stopRef(t_s);
        }
        break;
      }

      case Phase::STOPPING: {
        ref = stopRef(phase_time_);
        if (phase_time_ >= ramp_time_) {
          double th, th_d, th_dd, th_ddd;
          stopTheta(ramp_time_, th, th_d, th_dd, th_ddd);
          const RefState end = curveRef(th, 0.0, 0.0, 0.0);
          hold_ref_ = RefState{};
          hold_ref_.p = end.p;
          hold_ref_.yaw = last_cmd_yaw_;
          hold_valid_ = true;
          setPhase(Phase::HOLD);
          ref = hold_ref_;
        }
        break;
      }
    }

    publishRef(ref);
  }

  RefState trackRef(double t) const
  {
    double th, th_d, th_dd, th_ddd;
    trackTheta(t, th, th_d, th_dd, th_ddd);
    return curveRef(theta0_ + th, th_d, th_dd, th_ddd);
  }

  RefState stopRef(double t) const
  {
    double th, th_d, th_dd, th_ddd;
    stopTheta(t, th, th_d, th_dd, th_ddd);
    return curveRef(th, th_d, th_dd, th_ddd);
  }

  // ------------------------------------------------------------ publishing
  void publishRef(const RefState & ref)
  {
    // Final yaw-rate limiter: never command a yaw step faster than the limit,
    // and keep the published yaw_dot consistent with what is commanded.
    double yaw_cmd = ref.yaw;
    double yaw_dot_cmd = std::clamp(ref.yaw_dot, -yaw_rate_max_, yaw_rate_max_);
    if (have_last_yaw_) {
      const double dy = wrapPi(yaw_cmd - last_cmd_yaw_);
      const double max_step = yaw_rate_max_ * dt_;
      if (std::fabs(dy) > max_step) {
        yaw_cmd = wrapPi(last_cmd_yaw_ + std::clamp(dy, -max_step, max_step));
        yaw_dot_cmd = (dy > 0 ? yaw_rate_max_ : -yaw_rate_max_);
      }
    }
    last_cmd_yaw_ = yaw_cmd;
    have_last_yaw_ = true;

    mav_controllers_ros::msg::TargetCommand msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.position.x = ref.p.x;  msg.position.y = ref.p.y;  msg.position.z = ref.p.z;
    msg.velocity.x = ref.v.x;  msg.velocity.y = ref.v.y;  msg.velocity.z = ref.v.z;
    msg.acceleration.x = ref.a.x;  msg.acceleration.y = ref.a.y;  msg.acceleration.z = ref.a.z;
    msg.jerk.x = ref.j.x;  msg.jerk.y = ref.j.y;  msg.jerk.z = ref.j.z;
    msg.yaw = yaw_cmd;
    msg.yaw_dot = yaw_dot_cmd;
    msg.kx = {0.0, 0.0, 0.0};
    msg.kv = {0.0, 0.0, 0.0};
    msg.use_msg_gains_flags = mav_controllers_ros::msg::TargetCommand::USE_MSG_GAINS_NONE;
    setpoint_pub_->publish(msg);

    // RViz: current setpoint pose + rolling reference/actual traces (10 Hz).
    const auto q = yawToQuat(yaw_cmd);
    geometry_msgs::msg::PoseStamped sp;
    sp.header = msg.header;
    sp.pose.position = msg.position;
    sp.pose.orientation = q;
    setpoint_pose_pub_->publish(sp);
    appendToPath(ref_path_msg_, ref.p, q);
    if ((++viz_tick_ % std::max(1, static_cast<int>(0.1 / dt_))) == 0) {
      ref_path_msg_.header.stamp = msg.header.stamp;
      actual_path_msg_.header.stamp = msg.header.stamp;
      ref_path_pub_->publish(ref_path_msg_);
      actual_path_pub_->publish(actual_path_msg_);
    }
  }

  // Steady-rate health snapshot for a ground station. Observation only:
  // every value here is state the node already maintains, and publishing
  // it cannot change what the node commands.
  void publishHealth()
  {
    static const char * kPhaseNames[] = {"HOLD", "GOTO", "TRACK", "STOPPING"};

    std::string why;
    const bool is_engaged = engaged(why);

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "trajectory_test";
    st.hardware_id = get_name();

    if (!have_odom_) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      st.message = "no odometry";
    } else if (phase_ != Phase::HOLD) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st.message = std::string("flying: ") + kPhaseNames[static_cast<int>(phase_)];
    } else if (is_engaged) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st.message = "holding - ready to start";
    } else {
      // Not an error: sitting on the ground disengaged is the normal
      // state before a test. It just cannot start yet, and the reason
      // is the single most useful thing to show.
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "holding - cannot start: " + why;
    }

    auto add = [&st](const char * key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      st.values.push_back(kv);
    };
    auto num = [](double v, int prec = 2) {
      char buf2[48];
      std::snprintf(buf2, sizeof(buf2), "%.*f", prec, v);
      return std::string(buf2);
    };
    auto bl = [](bool v) { return std::string(v ? "true" : "false"); };

    add("phase", kPhaseNames[static_cast<int>(phase_)]);
    add("phase_time_s", num(phase_time_));
    add("engaged", bl(is_engaged));
    add("blocked_reason", is_engaged ? "" : why);

    // Each precondition separately, so the panel can show which one to fix
    // rather than only the first that failed.
    add("odom_fresh", bl(odomFresh()));
    add("offboard", bl(offboard_));
    add("armed", bl(armed_));
    add("motors_enabled", bl(motors_enabled_));

    add("trajectory_type", get_parameter("trajectory_type").as_string());
    add("yaw_mode", get_parameter("yaw_mode").as_string());
    add("speed_requested", num(get_parameter("speed").as_double()));
    add("speed_achieved", num(v_achieved_));
    add("accel_achieved", num(a_achieved_));
    add("entry_distance_m", num(entry_distance_));
    add("radius", num(get_parameter("circle.radius").as_double()));
    add("width", num(get_parameter("lemniscate.width").as_double()));
    add("last_plan_ok", bl(last_plan_ok_));
    add("last_plan_report", last_plan_report_);

    // Geofence: the limits plus where the vehicle actually is inside them.
    const bool fence_on = get_parameter("geofence.enable").as_bool();
    add("geofence_enabled", bl(fence_on));
    add("geofence_max_radius_xy", num(get_parameter("geofence.max_radius_xy").as_double()));
    add("geofence_min_z", num(get_parameter("geofence.min_z").as_double()));
    add("geofence_max_z", num(get_parameter("geofence.max_z").as_double()));
    if (have_odom_) {
      // The fence is measured from the position captured at the last start
      // trigger, which is only meaningful once a plan exists.
      const double dxy = std::hypot(odom_pos_.x - origin_.x, odom_pos_.y - origin_.y);
      add("dist_from_origin_xy", last_plan_ok_ ? num(dxy) : std::string("-"));
      add("altitude_m", num(odom_pos_.z));
    } else {
      add("dist_from_origin_xy", "-");
      add("altitude_m", "-");
    }

    health_pub_->publish(st);
  }

  void setPhase(Phase p)
  {
    if (p == phase_) return;
    phase_ = p;
    phase_time_ = 0.0;
    static const char * names[] = {"HOLD", "GOTO", "TRACK", "STOPPING"};
    std_msgs::msg::String s;
    s.data = names[static_cast<int>(p)];
    status_pub_->publish(s);
    RCLCPP_INFO(get_logger(), "Phase -> %s", s.data.c_str());
  }

  // ---------------------------------------------------------------- members
  enum class YawMode { HOLD, FIXED, TANGENT, CENTER };

  rclcpp::Publisher<mav_controllers_ros::msg::TargetCommand>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_, actual_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  nav_msgs::msg::Path ref_path_msg_, actual_path_msg_;
  int viz_tick_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  double dt_{0.01};

  // Vehicle state
  Vec3 odom_pos_;
  double odom_yaw_{0.0};
  rclcpp::Time last_odom_time_;
  bool have_odom_{false};
  bool offboard_{false};
  bool armed_{false};
  bool motors_enabled_{false};

  // Plan
  int traj_type_{1};
  YawMode yaw_mode_{YawMode::TANGENT};
  Vec3 origin_, center_;
  double origin_yaw_{0.0};
  double radius_{3.0}, width_{4.0};
  double omega_{0.0}, dir_{1.0}, theta0_{0.0};
  double ramp_time_{4.0};
  double v_max_{8.0}, a_max_{5.0}, j_max_{40.0}, yaw_rate_max_{1.0};
  double fixed_yaw_{0.0};

  // Phase machine
  Phase phase_{Phase::HOLD};
  double phase_time_{0.0};
  bool start_requested_{false};
  bool stop_requested_{false};
  bool auto_start_done_{false};
  bool traj_type_after_goto_hold_{false};

  // Hold / goto state
  RefState hold_ref_;
  bool hold_valid_{false};
  struct { Vec3 p; double yaw{0.0}; } goto_start_, goto_target_;
  double goto_T_{1.0};
  double stop_theta0_{0.0};

  // Last plan summary, surfaced on the health topic so a ground station
  // can see what the node actually committed to (derating included).
  std::string last_plan_report_{"no plan yet"};
  bool last_plan_ok_{false};
  double v_achieved_{0.0}, a_achieved_{0.0}, entry_distance_{0.0};

  // Yaw limiter memory
  double last_cmd_yaw_{0.0};
  bool have_last_yaw_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTestNode>());
  rclcpp::shutdown();
  return 0;
}
