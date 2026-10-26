// race_core: the single C++ process that drives the car (RACE_PLAN.md §5).
//
// mode (parameter):
//   reactive  follow-the-gap only (Plan B; the v0 behaviour)
//   race      particle filter on the race-day map + pure pursuit on the raceline (GB_TRACK), with
//             trailing behind an opponent on our line, and follow-the-gap (REACTIVE) whenever
//             localization is not healthy (start-up, after a hand reset, lost)
// Always on: deadman (joystick button held, unless require_deadman is false), scan and odometry
// watchdogs, emergency braking on time-to-collision in the swept corridor.
//
// Inputs: /scan, /odometry/filtered (robot_localization EKF), /joy, /initialpose (RViz).
// Outputs: /drive, TF map->odom, /race_core/telemetry, /race_core/pose, /race_core/raceline,
//          /race_core/particles, /race_core/lookahead.
// Runtime tuning: ros2 param set /race_core speed_scale 0.6   (also reactive_speed_scale, ...)
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "race_core/controller.hpp"
#include "race_core/follow_gap.hpp"
#include "race_core/geometry.hpp"
#include "race_core/occupancy_map.hpp"
#include "race_core/opponent.hpp"
#include "race_core/overtake.hpp"
#include "race_core/particle_filter.hpp"
#include "race_core/raceline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace race_core
{

enum class State : int {kStop = 0, kReactive = 1, kTrack = 2, kTrailing = 3, kOvertake = 4};

const char * stateName(State s)
{
  switch (s) {
    case State::kStop: return "STOP";
    case State::kReactive: return "REACTIVE";
    case State::kTrack: return "GB_TRACK";
    case State::kTrailing: return "TRAILING";
    case State::kOvertake: return "OVERTAKE";
  }
  return "?";
}

// /race_core/telemetry layout (Float32MultiArray), one message per scan:
//  0 t  1 state  2 engaged  3 x  4 y  5 yaw  6 v  7 s  8 d  9 v_ref  10 steer_cmd  11 speed_cmd
// 12 pf_match  13 pf_neff  14 pf_spread  15 ttc  16 lap_count  17 last_lap_s  18 pipeline_ms
// 19 opp_valid  20 opp_gap  21 opp_vs  22 localized  23 aeb_active  24 opp_camera_confirmed
// 25 camera_alive
constexpr int kTelemetrySize = 26;

struct OdomSample
{
  double t;
  Pose2D pose;
  double v;
  double wz;
};

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class RaceCoreNode : public rclcpp::Node
{
public:
  RaceCoreNode()
  : Node("race_core")
  {
    declareParameters();
    follow_gap_ = std::make_unique<FollowGap>(fg_);

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    telem_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/race_core/telemetry", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/race_core/pose", 10);
    particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/race_core/particles", 2);
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/race_core/lookahead", 5);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/race_core/raceline",
        rclcpp::QoS(1).transient_local());
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (mode_ == "race") {loadRaceAssets();}

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(*m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onOdom(*m);});
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      get_parameter("joy_topic").as_string(), rclcpp::QoS(10),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr m) {onJoy(*m);});
    // dpu_detector output: [latency_ms, n, (x1, y1, x2, y2, score, bearing, bearing_l, bearing_r) x n]
    dets_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>("/perception/camera_dets",
        rclcpp::QoS(5), [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
          last_dets_time_ = now();
          cam_bearings_.clear();
          const int n = m->data.size() >= 2 ? static_cast<int>(m->data[1]) : 0;
          for (int i = 0; i < n && 2 + 8 * i + 7 < static_cast<int>(m->data.size()); ++i) {
            const float * d = &m->data[2 + 8 * i];
            cam_bearings_.push_back({d[6], d[7]});   // bearing_left, bearing_right (left positive)
          }
        });
    initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(5),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        initLocalization({m->pose.pose.position.x, m->pose.pose.position.y,
            quatToYaw(m->pose.pose.orientation)}, "RViz /initialpose");
      });
    global_srv_ = create_service<std_srvs::srv::Trigger>("~/global_localization",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = startGlobalLocalization();
          res->message = res->success ? "global localization started" : "no map loaded";
        });
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & ps) {return onSetParameters(ps);});

    watchdog_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {onWatchdog();});
    viz_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {publishViz();});
    last_scan_time_ = now();
    RCLCPP_INFO(get_logger(), "race_core v1 up: mode=%s speed_scale=%.2f deadman=%s",
      mode_.c_str(), speed_scale_, require_deadman_ ? "required" : "OFF");
  }

private:
  // ------------------------------------------------------------------------------ parameters
  void declareParameters()
  {
    mode_ = declare_parameter("mode", std::string("reactive"));
    declare_parameter("scan_topic", std::string("/scan"));
    declare_parameter("odom_topic", std::string("/odometry/filtered"));
    declare_parameter("joy_topic", std::string("/joy"));
    drive_topic_ = declare_parameter("drive_topic", std::string("/drive"));
    map_yaml_ = declare_parameter("map_yaml", std::string(""));
    raceline_csv_ = declare_parameter("raceline_csv", std::string(""));
    raceline_direction_ = declare_parameter("raceline_direction", std::string("auto"));
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("frame_id", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);
    deskew_ = declare_parameter("deskew", true);
    scan_timeout_ = declare_parameter("scan_timeout", 0.15);
    odom_timeout_ = declare_parameter("odom_timeout", 0.2);
    speed_scale_ = declare_parameter("speed_scale", 0.5);
    reactive_speed_scale_ = declare_parameter("reactive_speed_scale", 1.0);
    require_deadman_ = declare_parameter("require_deadman", true);
    deadman_button_ = declare_parameter("deadman_button", 5);
    reset_button_ = declare_parameter("reset_button", 0);
    joy_timeout_ = declare_parameter("joy_timeout", 0.3);
    laser_ = {declare_parameter("laser_x", 0.27), declare_parameter("laser_y", 0.0),
      declare_parameter("laser_yaw", 0.0)};
    car_length_ = declare_parameter("car_length", 0.50);
    // per-sector speed factors on top of speed_scale: [s_start, s_end, factor, ...] (m along the
    // raceline, as printed by lap_report.py), e.g. slow one corner without slowing the whole lap
    setSectors(declare_parameter("sectors", std::vector<double>{}));

    fg_.bubble_radius = declare_parameter("follow_gap.bubble_radius", fg_.bubble_radius);
    fg_.preprocess_conv_size =
      declare_parameter("follow_gap.preprocess_conv_size", fg_.preprocess_conv_size);
    fg_.max_lidar_range = declare_parameter("follow_gap.max_lidar_range", fg_.max_lidar_range);
    fg_.speed_min = declare_parameter("follow_gap.speed_min", fg_.speed_min);
    fg_.speed_max = declare_parameter("follow_gap.speed_max", fg_.speed_max);
    fg_.steering_gain = declare_parameter("follow_gap.steering_gain", fg_.steering_gain);
    fg_.max_steer = declare_parameter("follow_gap.max_steer", fg_.max_steer);
    fg_.disparity_threshold =
      declare_parameter("follow_gap.disparity_threshold", fg_.disparity_threshold);
    fg_.vehicle_width = declare_parameter("follow_gap.vehicle_width", fg_.vehicle_width);
    fg_.disparity_margin = declare_parameter("follow_gap.disparity_margin", fg_.disparity_margin);
    fg_.medium_speed = declare_parameter("follow_gap.medium_speed", fg_.medium_speed);
    fg_.sharp_speed = declare_parameter("follow_gap.sharp_speed", fg_.sharp_speed);
    fg_.narrow_gap_speed = declare_parameter("follow_gap.narrow_gap_speed", fg_.narrow_gap_speed);

    pf_.particles = declare_parameter("pf.particles", pf_.particles);
    pf_beams_ = declare_parameter("pf.beams", 108);
    pf_.sigma_hit = declare_parameter("pf.sigma_hit", pf_.sigma_hit);
    pf_.z_hit = declare_parameter("pf.z_hit", pf_.z_hit);
    pf_.z_rand = declare_parameter("pf.z_rand", pf_.z_rand);
    pf_.squash = declare_parameter("pf.squash", pf_.squash);
    pf_.alpha_trans = declare_parameter("pf.alpha_trans", pf_.alpha_trans);
    pf_.alpha_rot_trans = declare_parameter("pf.alpha_rot_trans", pf_.alpha_rot_trans);
    pf_.alpha_rot = declare_parameter("pf.alpha_rot", pf_.alpha_rot);
    pf_.threads = declare_parameter("pf.threads", pf_.threads);
    pf_global_particles_ = declare_parameter("pf.global_particles", 20000);
    slip_gain_ = declare_parameter("pf.slip_gain", 2.0);
    init_pose_ = {declare_parameter("init_x", 0.0), declare_parameter("init_y", 0.0),
      declare_parameter("init_yaw", 0.0)};
    init_sigma_xy_ = declare_parameter("init_sigma_xy", 0.3);
    init_sigma_yaw_ = declare_parameter("init_sigma_yaw", 0.25);
    reset_sigma_xy_ = declare_parameter("reset_sigma_xy", 0.5);
    reset_sigma_yaw_ = declare_parameter("reset_sigma_yaw", 0.35);

    loc_score_good_ = declare_parameter("loc.score_good", 0.65);
    loc_score_lost_ = declare_parameter("loc.score_lost", 0.40);
    loc_good_scans_ = declare_parameter("loc.good_scans", 12);
    loc_lost_scans_ = declare_parameter("loc.lost_scans", 8);
    loc_max_spread_ = declare_parameter("loc.max_spread", 0.25);
    loc_max_lateral_ = declare_parameter("loc.max_lateral", 0.8);

    pp_.wheelbase = declare_parameter("pp.wheelbase", pp_.wheelbase);
    pp_.lookahead_base = declare_parameter("pp.lookahead_base", pp_.lookahead_base);
    pp_.lookahead_time = declare_parameter("pp.lookahead_time", pp_.lookahead_time);
    pp_.lookahead_min = declare_parameter("pp.lookahead_min", pp_.lookahead_min);
    pp_.lookahead_max = declare_parameter("pp.lookahead_max", pp_.lookahead_max);
    pp_.delay = declare_parameter("pp.delay", pp_.delay);
    pp_.max_steer_left = declare_parameter("pp.max_steer_left", pp_.max_steer_left);
    pp_.max_steer_right = declare_parameter("pp.max_steer_right", pp_.max_steer_right);
    pp_.speed_preview = declare_parameter("pp.speed_preview", pp_.speed_preview);
    pp_.lat_err_slow_start = declare_parameter("pp.lat_err_slow_start", pp_.lat_err_slow_start);
    pp_.lat_err_slow_gain = declare_parameter("pp.lat_err_slow_gain", pp_.lat_err_slow_gain);
    pp_.min_speed = declare_parameter("pp.min_speed", pp_.min_speed);

    aeb_enable_ = declare_parameter("aeb.enable", true);
    aeb_.ttc = declare_parameter("aeb.ttc", aeb_.ttc);
    aeb_.min_speed = declare_parameter("aeb.min_speed", aeb_.min_speed);
    aeb_.half_width = declare_parameter("aeb.half_width", aeb_.half_width);
    aeb_.margin = declare_parameter("aeb.margin", aeb_.margin);
    aeb_.front_offset = declare_parameter("aeb.front_offset", aeb_.front_offset);
    aeb_hold_ = declare_parameter("aeb.hold", 0.4);
    aeb_ttc_hard_ = declare_parameter("aeb.ttc_hard", 0.12);       // s, any point, even on the raceline
    aeb_foreign_dist_ = declare_parameter("aeb.foreign_dist", 0.15);  // m from mapped walls
    aeb_.wheelbase = pp_.wheelbase;

    opp_.min_wall_dist = declare_parameter("opp.min_wall_dist", opp_.min_wall_dist);
    opp_.max_range = declare_parameter("opp.max_range", opp_.max_range);
    opp_.min_points = declare_parameter("opp.min_points", opp_.min_points);
    trail_enable_ = declare_parameter("trail.enable", true);
    trail_gap_ = declare_parameter("trail.gap", 0.8);          // m bumper-to-bumper at standstill
    trail_time_gap_ = declare_parameter("trail.time_gap", 0.25);
    trail_kp_ = declare_parameter("trail.kp", 1.2);
    trail_min_gap_ = declare_parameter("trail.min_gap", 0.35);  // m bumper-to-bumper: stop
    trail_lane_ = declare_parameter("trail.lane_half_width", 0.45);
    overtake_mode_ = declare_parameter("overtake.mode", std::string("none"));  // none|spline|reactive
    overtake_ratio_ = declare_parameter("overtake.max_speed_ratio", 0.8);
    overtake_cap_ = declare_parameter("overtake.speed_cap", 2.5);
    overtake_after_ = declare_parameter("overtake.trail_time", 1.0);
    ot_.clearance = declare_parameter("overtake.clearance", ot_.clearance);
    ot_.wall_margin = declare_parameter("overtake.wall_margin", ot_.wall_margin);
    ot_.ramp_in = declare_parameter("overtake.ramp_in", ot_.ramp_in);
    ot_.ramp_out = declare_parameter("overtake.ramp_out", ot_.ramp_out);
    ot_.hold_after = declare_parameter("overtake.hold_after", ot_.hold_after);
    ot_.opp_length = car_length_;
    ot_abort_margin_ = declare_parameter("overtake.abort_margin", 0.08);
    ot_start_margin_ = declare_parameter("overtake.start_margin", 0.10);
    ot_boost_ = declare_parameter("overtake.speed_boost", 0.15);    // added to speed_scale while passing
    ot_cooldown_ = declare_parameter("overtake.cooldown", 2.0);     // s after an abort
    tracker_ = OpponentTracker(opp_);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> & ps)
  {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    for (const auto & p : ps) {
      const auto & n = p.get_name();
      if (n == "speed_scale") {speed_scale_ = p.as_double();}
      else if (n == "reactive_speed_scale") {reactive_speed_scale_ = p.as_double();}
      else if (n == "require_deadman") {require_deadman_ = p.as_bool();}
      else if (n == "pp.lookahead_base") {pp_.lookahead_base = p.as_double();}
      else if (n == "pp.lookahead_time") {pp_.lookahead_time = p.as_double();}
      else if (n == "pp.lookahead_min") {pp_.lookahead_min = p.as_double();}
      else if (n == "pp.lookahead_max") {pp_.lookahead_max = p.as_double();}
      else if (n == "pp.delay") {pp_.delay = p.as_double();}
      else if (n == "pp.min_speed") {pp_.min_speed = p.as_double();}
      else if (n == "aeb.enable") {aeb_enable_ = p.as_bool();}
      else if (n == "aeb.ttc") {aeb_.ttc = p.as_double();}
      else if (n == "trail.enable") {trail_enable_ = p.as_bool();}
      else if (n == "trail.gap") {trail_gap_ = p.as_double();}
      else if (n == "overtake.mode") {overtake_mode_ = p.as_string();}
      else if (n == "reset_button") {reset_button_ = static_cast<int>(p.as_int());}
      else if (n == "sectors") {setSectors(p.as_double_array());}
      else if (n == "deadman_button") {deadman_button_ = static_cast<int>(p.as_int());}
      else {continue;}
      RCLCPP_INFO(get_logger(), "param %s updated", n.c_str());
    }
    return r;
  }

  void setSectors(const std::vector<double> & v)
  {
    sectors_.clear();
    for (size_t i = 0; i + 3 <= v.size(); i += 3) {
      sectors_.push_back({v[i], v[i + 1], v[i + 2]});
    }
    if (!v.empty()) {
      RCLCPP_INFO(get_logger(), "%zu speed sector(s) set", sectors_.size());
    }
  }

  double sectorFactor(double s) const
  {
    double f = 1.0;
    for (const auto & sec : sectors_) {
      const bool inside = sec[0] <= sec[1] ? (s >= sec[0] && s <= sec[1]) : (s >= sec[0] || s <= sec[1]);
      if (inside) {f = std::min(f, sec[2]);}
    }
    return f;
  }

  // ------------------------------------------------------------------------ race assets
  void loadRaceAssets()
  {
    std::string err;
    if (map_yaml_.empty() || !map_.load(map_yaml_, &err)) {
      RCLCPP_ERROR(get_logger(), "race mode without a usable map (%s) -> REACTIVE only",
        map_yaml_.empty() ? "map_yaml not set" : err.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "map %s: %dx%d @ %.3f m", map_yaml_.c_str(), map_.width(),
      map_.height(), map_.resolution());
    pf_filter_ = std::make_unique<ParticleFilter>(map_, pf_);
    if (raceline_csv_.empty() || !raceline_.load(raceline_csv_, &err)) {
      RCLCPP_ERROR(get_logger(), "no usable raceline (%s) -> REACTIVE only",
        raceline_csv_.empty() ? "raceline_csv not set" : err.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "raceline %s: %d points, %.1f m, v %.1f..%.1f m/s (x%.2f)",
        raceline_csv_.c_str(), raceline_.size(), raceline_.length(), minV(), maxV(), speed_scale_);
      if (raceline_direction_ == "reverse") {reverseRaceline();}
      direction_checked_ = raceline_direction_ != "auto";
      publishPath();
    }
    initLocalization(init_pose_, "start pose parameters");
  }

  double minV() const
  {
    double v = 1e9;
    for (const auto & p : raceline_.points()) {v = std::min(v, p.v);}
    return v;
  }
  double maxV() const
  {
    double v = 0;
    for (const auto & p : raceline_.points()) {v = std::max(v, p.v);}
    return v;
  }

  // Prefer make_raceline.py's <name>_rev.csv (speed profile computed for that direction).
  void reverseRaceline()
  {
    std::string rev = raceline_csv_;
    const auto dot = rev.rfind(".csv");
    if (dot != std::string::npos) {
      rev.insert(dot, rev.find("_rev.csv") == std::string::npos ? "_rev" : "");
      if (rev.find("_rev_rev") != std::string::npos) {rev.replace(rev.find("_rev_rev"), 8, "");}
    }
    Raceline r;
    std::string err;
    if (rev != raceline_csv_ && r.load(rev, &err)) {
      raceline_ = r;
      RCLCPP_WARN(get_logger(), "raceline direction reversed: using %s", rev.c_str());
    } else {
      auto pts = raceline_.points();
      std::reverse(pts.begin(), pts.end());
      for (auto & p : pts) {
        p.psi = normalizeAngle(p.psi + M_PI);
        p.kappa = -p.kappa;
        std::swap(p.w_left, p.w_right);
      }
      raceline_.setPoints(std::move(pts));
      RCLCPP_WARN(get_logger(), "raceline direction reversed in memory (no %s)", rev.c_str());
    }
    line_hint_ = -1;
    publishPath();
  }

  void initLocalization(const Pose2D & pose, const char * why)
  {
    if (!pf_filter_) {return;}
    pf_filter_->initGaussian(pose, init_sigma_xy_, init_sigma_yaw_);
    have_pf_odom_ = false;
    localized_ = false;
    good_count_ = bad_count_ = 0;
    global_mode_ = false;
    line_hint_ = -1;
    RCLCPP_INFO(get_logger(), "localization initialised at (%.2f, %.2f, %.0f deg) from %s",
      pose.x, pose.y, pose.yaw * 180.0 / M_PI, why);
  }

  bool startGlobalLocalization()
  {
    if (!pf_filter_) {return false;}
    pf_filter_->initGlobal(pf_global_particles_);
    have_pf_odom_ = false;
    localized_ = false;
    good_count_ = bad_count_ = 0;
    global_mode_ = true;
    line_hint_ = -1;
    RCLCPP_WARN(get_logger(), "global localization with %d particles", pf_global_particles_);
    return true;
  }

  // -------------------------------------------------------------------------- odometry
  void onOdom(const nav_msgs::msg::Odometry & m)
  {
    OdomSample s;
    s.t = rclcpp::Time(m.header.stamp).seconds();
    s.pose = {m.pose.pose.position.x, m.pose.pose.position.y, quatToYaw(m.pose.pose.orientation)};
    s.v = m.twist.twist.linear.x;
    s.wz = m.twist.twist.angular.z;
    if (!odom_.empty() && s.t < odom_.back().t) {odom_.clear();}   // time jumped back (bag loop)
    odom_.push_back(s);
    while (odom_.size() > 2 && odom_.back().t - odom_.front().t > 2.0) {odom_.pop_front();}
    last_odom_wall_ = now();
    if (lineFollowing()) {controlStep(s);}
  }

  // States driven by the raceline controller (a spline overtake follows the shifted raceline;
  // the reactive overtake is driven by follow-the-gap from the scan callback).
  bool lineFollowing() const
  {
    return state_ == State::kTrack || state_ == State::kTrailing ||
           (state_ == State::kOvertake && overtake_mode_ != "reactive");
  }

  bool odomFresh() {return !odom_.empty() && (now() - last_odom_wall_).seconds() < odom_timeout_;}

  bool odomAt(double t, OdomSample & out) const
  {
    if (odom_.empty()) {return false;}
    if (t >= odom_.back().t) {
      const auto & b = odom_.back();
      const double dt = t - b.t;
      if (dt > 0.15) {return false;}
      out = b;
      out.pose.x += b.v * dt * std::cos(b.pose.yaw);
      out.pose.y += b.v * dt * std::sin(b.pose.yaw);
      out.pose.yaw = normalizeAngle(b.pose.yaw + b.wz * dt);
      out.t = t;
      return true;
    }
    if (t <= odom_.front().t) {
      if (odom_.front().t - t > 0.15) {return false;}
      out = odom_.front();
      return true;
    }
    auto it = std::lower_bound(odom_.begin(), odom_.end(), t,
        [](const OdomSample & a, double tt) {return a.t < tt;});
    const auto & b = *it;
    const auto & a = *(it - 1);
    const double f = (t - a.t) / std::max(1e-9, b.t - a.t);
    out.t = t;
    out.pose.x = a.pose.x + f * (b.pose.x - a.pose.x);
    out.pose.y = a.pose.y + f * (b.pose.y - a.pose.y);
    out.pose.yaw = normalizeAngle(a.pose.yaw + f * normalizeAngle(b.pose.yaw - a.pose.yaw));
    out.v = a.v + f * (b.v - a.v);
    out.wz = a.wz + f * (b.wz - a.wz);
    return true;
  }

  // ----------------------------------------------------------------------------- scans
  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    const auto t0 = std::chrono::steady_clock::now();
    last_scan_time_ = now();
    // stamp = first beam (ROS convention); the points are expressed at mid-scan
    const double t_scan = rclcpp::Time(scan.header.stamp).seconds() +
      0.5 * std::max<size_t>(scan.ranges.size(), 1) * std::max(0.0f, scan.time_increment);
    buildPoints(scan);

    double pf_ms = 0.0;
    if (pf_filter_) {
      const auto p0 = std::chrono::steady_clock::now();
      localize(t_scan);
      pf_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - p0).count();
    }
    const bool on_map = localized_ && !raceline_.empty();
    foreign_pts_.clear();
    if (on_map) {
      const Pose2D pose = currentPose();
      const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
      map_pts_.resize(aeb_pts_.size());
      for (size_t i = 0; i + 1 < aeb_pts_.size(); i += 2) {
        map_pts_[i] = static_cast<float>(pose.x + c * aeb_pts_[i] - s * aeb_pts_[i + 1]);
        map_pts_[i + 1] = static_cast<float>(pose.y + s * aeb_pts_[i] + c * aeb_pts_[i + 1]);
        if (map_.distanceAt(map_pts_[i], map_pts_[i + 1]) > aeb_foreign_dist_) {
          foreign_pts_.push_back(aeb_pts_[i]);
          foreign_pts_.push_back(aeb_pts_[i + 1]);
        }
      }
      const auto obs = detectObstacles(map_pts_, pose.x, pose.y, map_, raceline_, line_hint_, opp_);
      tracker_.update(obs, t_scan, ego_s_, raceline_, 6.0, state_ == State::kOvertake ? 2.5 : 0.5);
      updateCameraConfirmation(pose);
    } else {
      tracker_.reset();
    }

    // Emergency braking. On the raceline the walls are the planner's job (the line keeps its margin),
    // so only points the map does not explain (cars, people, boxes) count, plus every point at a
    // last-resort TTC. Off the raceline (REACTIVE) every point counts.
    const double v_now = odom_.empty() ? last_speed_cmd_ : odom_.back().v;
    const double ttc_all = minTimeToCollision(aeb_pts_, v_now, last_steer_cmd_, aeb_);
    const bool tracking = on_map && lineFollowing();
    ttc_ = tracking ? minTimeToCollision(foreign_pts_, v_now, last_steer_cmd_, aeb_) : ttc_all;
    const bool brake = ttc_ < aeb_.ttc || (tracking && ttc_all < aeb_ttc_hard_);
    if (aeb_enable_ && brake && engaged()) {
      if (!aebActive()) {
        ++aeb_events_;
        const Pose2D pose = currentPose();
        RCLCPP_WARN(get_logger(), "EMERGENCY BRAKE: ttc %.2f s (all points %.2f s) at %.1f m/s, "
          "%s, pose (%.2f, %.2f) (event %d)", ttc_, ttc_all, v_now, stateName(state_), pose.x, pose.y,
          aeb_events_);
      }
      aeb_until_ = now() + rclcpp::Duration::from_seconds(aeb_hold_);
    }

    updateState();
    if (state_ == State::kReactive || (state_ == State::kOvertake && overtake_mode_ == "reactive")) {
      const auto cmd = follow_gap_->compute(scan.ranges, scan.angle_min, scan.angle_increment);
      if (cmd) {
        double v = cmd->speed * reactive_speed_scale_;
        if (state_ == State::kOvertake) {v = std::min(v, overtake_cap_);}
        publishDrive(cmd->steering, v);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Unusable scan, stopping");
        publishDrive(0.0, 0.0);
      }
    }
    if (pf_filter_ && publish_tf_) {publishMapOdom(scan.header.stamp);}
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    publishTelemetry(t_scan, ms);
    recordLatency(ms, pf_ms);
  }

  // Beam end points in base_link at the scan's middle time. With deskewing on, each beam is moved by
  // the motion between its own time and mid-scan (constant v, yaw rate from the latest odometry):
  // a Hokuyo scan spans ~19 ms, i.e. ~10 cm and up to ~3 deg at race speed.
  void buildPoints(const sensor_msgs::msg::LaserScan & scan)
  {
    pf_eps_.clear();
    aeb_pts_.clear();
    const int n = static_cast<int>(scan.ranges.size());
    const int stride = std::max(1, n / std::max(1, pf_beams_));
    const double rmin = std::max<double>(scan.range_min, 0.05);
    const double cl = std::cos(laser_.yaw), sl = std::sin(laser_.yaw);
    const double mid = 0.5 * (n - 1);
    double v = 0.0, w = 0.0;
    const bool deskew = deskew_ && scan.time_increment > 0.0f && odomFresh();
    if (deskew) {
      v = odom_.back().v;
      w = odom_.back().wz;
    }
    for (int i = 0; i < n; ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < rmin || r > scan.range_max) {continue;}
      const double a = scan.angle_min + i * scan.angle_increment;
      const double lx = r * std::cos(a), ly = r * std::sin(a);
      double x = laser_.x + cl * lx - sl * ly;
      double y = laser_.y + sl * lx + cl * ly;
      if (deskew) {   // robot pose at this beam's time, relative to mid-scan
        const double dt = (i - mid) * scan.time_increment;
        const double dyaw = w * dt, c = std::cos(dyaw), s = std::sin(dyaw);
        const double tx = v * dt * std::cos(0.5 * dyaw), ty = v * dt * std::sin(0.5 * dyaw);
        const double xr = c * x - s * y + tx, yr = s * x + c * y + ty;
        x = xr;
        y = yr;
      }
      if (i % 2 == 0) {
        aeb_pts_.push_back(static_cast<float>(x));
        aeb_pts_.push_back(static_cast<float>(y));
      }
      if (i % stride == 0) {
        pf_eps_.push_back(static_cast<float>(x));
        pf_eps_.push_back(static_cast<float>(y));
      }
    }
  }

  // ----------------------------------------------------------------- camera confirmation
  // The LiDAR track counts as a camera-confirmed car when a fresh DPU detection's bearing span covers
  // the track's bearing (+-4 deg). Labelling only: racing never depends on the camera.
  bool cameraAlive() {return last_dets_time_.nanoseconds() > 0 && (now() - last_dets_time_).seconds() < 0.5;}

  void updateCameraConfirmation(const Pose2D & pose)
  {
    const auto & tr = tracker_.track();
    if (!tr.valid || !cameraAlive()) {
      opp_confirmed_ = false;
      return;
    }
    const double b = normalizeAngle(std::atan2(tr.last.y - pose.y, tr.last.x - pose.x) - pose.yaw);
    const double tol = 4.0 * M_PI / 180.0;
    bool hit = false;
    for (const auto & cb : cam_bearings_) {   // left edge > right edge (left positive)
      if (b <= cb[0] + tol && b >= cb[1] - tol) {hit = true;}
    }
    if (hit && !opp_confirmed_) {
      RCLCPP_INFO(get_logger(), "opponent confirmed by the camera (bearing %+.0f deg, %.1f m ahead)",
        b * 180.0 / M_PI, raceline_.ds(ego_s_, tr.s));
    }
    opp_confirmed_ = hit;
  }

  // ----------------------------------------------------------------------- localization
  void localize(double t_scan)
  {
    OdomSample o;
    if (odom_.empty() || !odomFresh()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "race mode needs odometry on %s (start the EKF: ./race.sh ekf); driving REACTIVE until then",
        get_parameter("odom_topic").as_string().c_str());
    }
    if (!odomAt(t_scan, o)) {
      // Without odometry the filter cannot follow a moving car; hold the last estimate.
      if (++bad_count_ >= loc_lost_scans_ && localized_) {
        localized_ = false;
        RCLCPP_WARN(get_logger(), "localization suspended: no odometry at scan time");
      }
      return;
    }
    const Pose2D delta = have_pf_odom_ ? between(pf_odom_, o.pose) : Pose2D{};
    // slip: gyro yaw rate vs the yaw rate the commanded steering would give
    const double wz_ack = o.v * std::tan(last_steer_cmd_) / pp_.wheelbase;
    const double slip = std::clamp(1.0 + slip_gain_ * std::fabs(o.wz - wz_ack), 1.0, 3.0);
    pf_filter_->predict(delta, slip);
    pf_odom_ = o.pose;
    have_pf_odom_ = true;
    if (!pf_filter_->update(pf_eps_)) {return;}
    const Pose2D est = pf_filter_->estimate();
    map_odom_ = compose(est, inverse(o.pose));
    have_map_odom_ = true;

    if (global_mode_ && pf_filter_->spreadXY() < 0.3 && pf_filter_->spreadYaw() < 0.3) {
      pf_filter_->shrinkTo(pf_.particles);
      global_mode_ = false;
      RCLCPP_INFO(get_logger(), "global localization converged at (%.2f, %.2f)", est.x, est.y);
    }
    const double score = pf_filter_->matchScore();
    const double spread = pf_filter_->spreadXY();
    if (score >= loc_score_good_ && spread < loc_max_spread_ && !global_mode_) {
      ++good_count_;
      bad_count_ = 0;
    } else if (score < loc_score_lost_ || spread > 2.0 * loc_max_spread_) {
      ++bad_count_;
      good_count_ = 0;
    }
    if (!localized_ && good_count_ >= loc_good_scans_) {
      localized_ = true;
      RCLCPP_INFO(get_logger(), "localized (match %.2f, spread %.2f m)", score, spread);
    } else if (localized_ && bad_count_ >= loc_lost_scans_) {
      localized_ = false;
      RCLCPP_WARN(get_logger(), "LOCALIZATION LOST (match %.2f, spread %.2f m) -> REACTIVE", score,
        spread);
    }
    if (localized_ && !direction_checked_ && !raceline_.empty()) {checkDirection(est);}
  }

  void checkDirection(const Pose2D & est)
  {
    const Projection pr = raceline_.project(est.x, est.y);
    const double diff = std::fabs(normalizeAngle(est.yaw - pr.psi));
    if (diff > 1.05 && diff < 2.09) {return;}   // sideways to the line: decide later
    direction_checked_ = true;
    if (diff >= 2.09) {reverseRaceline();}
    RCLCPP_INFO(get_logger(), "raceline direction %s (car heading vs line %.0f deg)",
      diff >= 2.09 ? "REVERSED to match the car" : "matches the car", diff * 180.0 / M_PI);
  }

  Pose2D currentPose() const
  {
    if (odom_.empty()) {return pf_filter_ ? pf_filter_->estimate() : Pose2D{};}
    return compose(map_odom_, odom_.back().pose);
  }

  // --------------------------------------------------------------------- state machine
  void updateState()
  {
    State next = State::kReactive;
    double lateral = 0.0;
    if (mode_ == "race" && localized_ && !raceline_.empty() && odomFresh() && have_map_odom_) {
      const Pose2D pose = currentPose();
      const Projection pr = raceline_.project(pose.x, pose.y, line_hint_);
      lateral = pr.d;
      ego_s_ = pr.s;
      if (std::fabs(lateral) <= loc_max_lateral_) {next = State::kTrack;}
    }
    if (next == State::kTrack) {
      const auto & tr = tracker_.track();
      opp_gap_ = 1e9;
      bool on_line = false;
      if (tr.valid && tr.hits >= 3) {
        opp_gap_ = raceline_.ds(ego_s_, tr.s) - car_length_;   // bumper to bumper (approx.)
        // compare with where we WILL be at the opponent's position (the raceline, or the overtake
        // offset there), not with our current offset: ramping back onto the line must not miss it
        const double planned_d = ot_plan_.side != 0 ? overtakeOffset(raceline_, ot_plan_, tr.s) : 0.0;
        on_line = std::fabs(tr.d - planned_d) < trail_lane_ || std::fabs(tr.d - lateral) < trail_lane_;
      }
      const bool trailing = trail_enable_ && on_line && opp_gap_ > -0.3 && opp_gap_ < 5.0;
      if (trailing) {
        next = State::kTrailing;
        trail_since_ = (state_ == State::kTrailing || state_ == State::kOvertake) ? trail_since_ : now();
        const double v_ref = raceline_.at(ego_s_).v * speed_scale_;
        const bool slow = tr.vs < overtake_ratio_ * v_ref;
        const bool ready = slow && (now() - trail_since_).seconds() > overtake_after_ && opp_gap_ < 2.5;
        if (overtake_mode_ == "reactive" && ready) {
          next = State::kOvertake;
        } else if (overtake_mode_ == "spline" && ready && state_ != State::kOvertake &&
          (now() - ot_abort_time_).seconds() > ot_cooldown_)
        {
          const double v_pass = passSpeed();
          const OvertakePlan plan = planOvertake(raceline_, ego_s_, v_pass, tr, 0, ot_);
          if (plan.feasible && plan.margin >= ot_start_margin_) {
            ot_plan_ = plan;
            ot_line_ = shiftedLine(raceline_, ot_plan_);
            next = State::kOvertake;
            RCLCPP_INFO(get_logger(), "OVERTAKE on the %s: offset %+.2f m, spare room %.2f m, opponent "
              "%.1f m/s vs our %.1f", plan.side > 0 ? "left" : "right", plan.d_target, plan.margin, tr.vs,
              v_ref);
          }
        }
      }
      if (state_ == State::kOvertake && overtake_mode_ == "reactive" && tr.valid &&
        opp_gap_ > -car_length_ - 0.3 && opp_gap_ < 3.0)
      {
        next = State::kOvertake;   // stay until the opponent is clearly behind us
      }
      if (state_ == State::kOvertake && overtake_mode_ == "spline") {
        next = splineOvertakeStep(tr);
      }
    }
    // a plan stays active (ramping back to the raceline) until we pass its end
    if (ot_plan_.side != 0 && next != State::kOvertake &&
      raceline_.ds(ot_plan_.s_end, ego_s_) > 0.0 && raceline_.ds(ot_plan_.s_end, ego_s_) < 5.0)
    {
      ot_plan_ = OvertakePlan();
    }
    if (next == State::kReactive) {ot_plan_ = OvertakePlan();}
    if (next != state_) {
      RCLCPP_INFO(get_logger(), "state %s -> %s", stateName(state_), stateName(next));
      if (next == State::kReactive) {follow_gap_->reset();}
      state_ = next;
    }
  }

  // Our mean speed over the next 8 m while passing (boosted scale); stable from scan to scan.
  double passSpeed() const
  {
    double sum = 0.0;
    int n = 0;
    for (double u = 0.0; u <= 8.0; u += 0.5, ++n) {sum += raceline_.at(ego_s_ + u).v;}
    return sum / std::max(1, n) * std::min(1.0, speed_scale_ + ot_boost_);
  }

  // While overtaking: re-plan on the same side each scan; abort (ramp back now, then trail) if the
  // gap closes on our side; finish once we are past the opponent.
  State splineOvertakeStep(const OpponentTrack & tr)
  {
    if (raceline_.ds(ot_plan_.s_release, ego_s_) > 0.0) {
      RCLCPP_INFO(get_logger(), "overtake done");
      return State::kTrack;
    }
    if (!tr.valid) {return State::kOvertake;}    // out of sight beside/behind us: keep the plan
    const OvertakePlan plan = planOvertake(raceline_, ego_s_, passSpeed(), tr, ot_plan_.side, ot_, &ot_plan_);
    if (plan.feasible || plan.margin > -ot_abort_margin_) {   // hysteresis: committed is committed
      ot_plan_ = plan;
      ot_line_ = shiftedLine(raceline_, ot_plan_);
      return State::kOvertake;
    }
    // abort: start moving back from here
    ot_abort_time_ = now();
    ot_plan_.s_release = ego_s_;
    ot_plan_.s_end = raceline_.wrapS(ego_s_ + ot_.ramp_out);
    ot_line_ = shiftedLine(raceline_, ot_plan_);
    RCLCPP_WARN(get_logger(), "overtake aborted (no room on the %s: spare %.2f m, opponent d %+.2f, "
      "s ahead %.2f), trailing", ot_plan_.side > 0 ? "left" : "right", plan.margin, tr.d,
      raceline_.ds(ego_s_, tr.s));
    return State::kTrailing;
  }

  // ------------------------------------------------------------------------- control
  void controlStep(const OdomSample & o)
  {
    if (raceline_.empty() || !have_map_odom_) {return;}
    const Pose2D pose = compose(map_odom_, o.pose);
    const Raceline & line = ot_plan_.side != 0 ? ot_line_ : raceline_;
    const double scale = state_ == State::kOvertake ? std::min(1.0, speed_scale_ + ot_boost_) : speed_scale_;
    const auto out = purePursuit(line, pose, o.v, o.wz, line_hint_, scale, pp_);
    line_hint_ = out.proj.index;
    const double sector = sectorFactor(ot_plan_.side != 0 ? ego_s_ : out.proj.s);
    double speed = std::max(out.speed * sector, std::min(out.speed, pp_.min_speed));
    v_ref_ = out.v_ref * sector;
    if (state_ == State::kTrailing) {
      const auto & tr = tracker_.track();
      // gap on the global raceline (out.proj.s is on the shifted line while a plan ramps back)
      const double s_now = ot_plan_.side != 0 ? ego_s_ : out.proj.s;
      const double gap = raceline_.ds(s_now, tr.s) - car_length_;
      const double gap_des = trail_gap_ + trail_time_gap_ * std::max(0.0, tr.vs);
      double v_trail = std::max(0.0, tr.vs + trail_kp_ * (gap - gap_des));
      if (gap < trail_min_gap_) {v_trail = 0.0;}
      speed = std::min(speed, v_trail);
    }
    lapTiming(ot_plan_.side != 0 ? ego_s_ : out.proj.s);
    lookahead_[0] = out.target_x;
    lookahead_[1] = out.target_y;
    publishDrive(out.steering, speed);
  }

  void lapTiming(double s)
  {
    const double L = raceline_.length();
    const double t = now().seconds();
    if (have_last_s_ && last_s_ > 0.75 * L && s < 0.25 * L) {   // crossed s = 0 going forward
      if (lap_start_ > 0) {
        last_lap_ = t - lap_start_;
        ++lap_count_;
        RCLCPP_INFO(get_logger(), "LAP %d: %.2f s (scale %.2f, %d AEB events)", lap_count_, last_lap_,
          speed_scale_, aeb_events_);
      }
      lap_start_ = t;
    }
    last_s_ = s;
    have_last_s_ = true;
  }

  // ---------------------------------------------------------------------- joystick
  void onJoy(const sensor_msgs::msg::Joy & j)
  {
    last_joy_ = now();
    const auto btn = [&](int i) {return i >= 0 && i < static_cast<int>(j.buttons.size()) && j.buttons[i];};
    const bool dm = btn(deadman_button_);
    if (dm != deadman_) {
      RCLCPP_INFO(get_logger(), "deadman %s", dm ? "HELD: autonomous driving" : "released: stop");
    }
    deadman_ = dm;
    const bool rs = btn(reset_button_);
    if (rs && !reset_pressed_ && pf_filter_) {
      // hand reset: the car was put back near where it was; re-seed around the last estimate
      Pose2D last = have_map_odom_ ? currentPose() : init_pose_;
      if (!raceline_.empty()) {   // the drill puts the car down facing the race direction
        last.yaw = raceline_.project(last.x, last.y).psi;
      }
      pf_filter_->initGaussian(last, reset_sigma_xy_, reset_sigma_yaw_);
      have_pf_odom_ = false;
      localized_ = false;
      good_count_ = bad_count_ = 0;
      line_hint_ = -1;
      RCLCPP_WARN(get_logger(), "RESET: re-localizing around (%.2f, %.2f); REACTIVE until healthy",
        last.x, last.y);
    }
    reset_pressed_ = rs;
  }

  bool engaged()
  {
    if (!require_deadman_) {return true;}
    return deadman_ && (now() - last_joy_).seconds() < joy_timeout_;
  }

  bool aebActive() {return aeb_until_.nanoseconds() > 0 && now() < aeb_until_;}

  // ------------------------------------------------------------------------ output
  void publishDrive(double steering, double speed)
  {
    const bool eng = engaged();
    if (!eng) {speed = 0.0;}
    if (aebActive()) {speed = 0.0;}
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.drive.steering_angle = static_cast<float>(steering);
    msg.drive.speed = static_cast<float>(speed);
    drive_pub_->publish(msg);
    last_steer_cmd_ = steering;
    last_speed_cmd_ = speed;
  }

  void onWatchdog()
  {
    const double since = (now() - last_scan_time_).seconds();
    if (since > scan_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No scan for %.2f s, stopping", since);
      follow_gap_->reset();
      publishDrive(0.0, 0.0);
    }
    if (lineFollowing() && !odomFresh()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "odometry stale -> stopping until next scan");
      publishDrive(last_steer_cmd_, 0.0);
    }
  }

  void publishMapOdom(const builtin_interfaces::msg::Time & stamp)
  {
    if (!have_map_odom_) {return;}
    geometry_msgs::msg::TransformStamped tf;
    // future-date slightly (like AMCL's transform_tolerance) so consumers can use the latest odom
    tf.header.stamp = rclcpp::Time(stamp) + rclcpp::Duration::from_seconds(0.1);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = map_odom_.x;
    tf.transform.translation.y = map_odom_.y;
    tf.transform.rotation = yawToQuat(map_odom_.yaw);
    tf_pub_->sendTransform(tf);
  }

  void publishTelemetry(double t, double ms)
  {
    std_msgs::msg::Float32MultiArray m;
    m.data.assign(kTelemetrySize, 0.0f);
    const Pose2D pose = have_map_odom_ ? currentPose() : Pose2D{};
    const auto & tr = tracker_.track();
    double d = 0.0;
    if (!raceline_.empty() && have_map_odom_) {d = raceline_.project(pose.x, pose.y, line_hint_).d;}
    const float vals[kTelemetrySize] = {
      static_cast<float>(t), static_cast<float>(static_cast<int>(state_)),
      static_cast<float>(engaged()), static_cast<float>(pose.x), static_cast<float>(pose.y),
      static_cast<float>(pose.yaw), static_cast<float>(odom_.empty() ? 0.0 : odom_.back().v),
      static_cast<float>(ego_s_), static_cast<float>(d), static_cast<float>(v_ref_),
      static_cast<float>(last_steer_cmd_), static_cast<float>(last_speed_cmd_),
      static_cast<float>(pf_filter_ ? pf_filter_->matchScore() : 0.0),
      static_cast<float>(pf_filter_ ? pf_filter_->neffRatio() : 0.0),
      static_cast<float>(pf_filter_ ? pf_filter_->spreadXY() : 0.0),
      static_cast<float>(std::min(ttc_, 99.0)), static_cast<float>(lap_count_),
      static_cast<float>(last_lap_), static_cast<float>(ms), static_cast<float>(tr.valid),
      static_cast<float>(std::min(opp_gap_, 99.0)), static_cast<float>(tr.vs),
      static_cast<float>(localized_), static_cast<float>(aebActive()),
      static_cast<float>(opp_confirmed_), static_cast<float>(cameraAlive())};
    std::copy(vals, vals + kTelemetrySize, m.data.begin());
    telem_pub_->publish(m);

    if ((now() - last_status_).seconds() > 1.0) {
      last_status_ = now();
      RCLCPP_INFO(get_logger(),
        "%s%s | pose (%.2f, %.2f) d %+.2f | v %.2f ref %.2f cmd %.2f | match %.2f spread %.2f | "
        "ttc %.1f | laps %d last %.2f s",
        stateName(state_), engaged() ? "" : " (deadman released)", pose.x, pose.y, d,
        odom_.empty() ? 0.0 : odom_.back().v, v_ref_, last_speed_cmd_,
        pf_filter_ ? pf_filter_->matchScore() : 0.0, pf_filter_ ? pf_filter_->spreadXY() : 0.0,
        std::min(ttc_, 99.0), lap_count_, last_lap_);
    }
  }

  void publishViz()
  {
    if (!pf_filter_ || !have_map_odom_) {return;}
    const auto stamp = now();
    const Pose2D pose = currentPose();
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = map_frame_;
    ps.pose.position.x = pose.x;
    ps.pose.position.y = pose.y;
    ps.pose.orientation = yawToQuat(pose.yaw);
    pose_pub_->publish(ps);
    if (particles_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::PoseArray pa;
      pa.header = ps.header;
      const auto & P = pf_filter_->particles();
      const size_t step = std::max<size_t>(1, P.size() / 300);
      for (size_t i = 0; i < P.size(); i += step) {
        geometry_msgs::msg::Pose p;
        p.position.x = P[i].x;
        p.position.y = P[i].y;
        p.orientation = yawToQuat(P[i].yaw);
        pa.poses.push_back(p);
      }
      particles_pub_->publish(pa);
    }
    geometry_msgs::msg::PointStamped la;
    la.header = ps.header;
    la.point.x = lookahead_[0];
    la.point.y = lookahead_[1];
    lookahead_pub_->publish(la);
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (const auto & p : raceline_.points()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation = yawToQuat(p.psi);
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  // p50/p99 of the scan pipeline and the particle filter every 200 scans (~5 s).
  void recordLatency(double ms, double pf_ms)
  {
    latencies_.push_back(ms);
    pf_latencies_.push_back(pf_ms);
    if (latencies_.size() < 200) {return;}
    std::sort(latencies_.begin(), latencies_.end());
    std::sort(pf_latencies_.begin(), pf_latencies_.end());
    RCLCPP_INFO(get_logger(), "scan pipeline: p50 %.2f ms, p99 %.2f ms, max %.2f ms (pf p50 %.2f ms)",
      latencies_[latencies_.size() / 2], latencies_[latencies_.size() * 99 / 100], latencies_.back(),
      pf_latencies_[pf_latencies_.size() / 2]);
    latencies_.clear();
    pf_latencies_.clear();
  }

  // modules
  FollowGapParams fg_;
  std::unique_ptr<FollowGap> follow_gap_;
  OccupancyMap map_;
  PFParams pf_;
  std::unique_ptr<ParticleFilter> pf_filter_;
  Raceline raceline_;
  PurePursuitParams pp_;
  AebParams aeb_;
  OpponentParams opp_;
  OpponentTracker tracker_;
  OvertakeParams ot_;
  OvertakePlan ot_plan_;
  Raceline ot_line_;

  // ROS
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr telem_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr dets_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr watchdog_, viz_timer_;

  // parameters
  std::string mode_, drive_topic_, map_yaml_, raceline_csv_, raceline_direction_;
  std::string map_frame_, odom_frame_, base_frame_, overtake_mode_;
  bool publish_tf_ = true, require_deadman_ = true, aeb_enable_ = true, trail_enable_ = true;
  bool deskew_ = true;
  double scan_timeout_ = 0.15, odom_timeout_ = 0.2, speed_scale_ = 0.5, reactive_speed_scale_ = 1.0;
  double joy_timeout_ = 0.3, car_length_ = 0.5, slip_gain_ = 2.0, aeb_hold_ = 0.4;
  double aeb_ttc_hard_ = 0.12, aeb_foreign_dist_ = 0.15, ot_abort_margin_ = 0.08;
  double ot_start_margin_ = 0.05, ot_boost_ = 0.15, ot_cooldown_ = 2.0;
  rclcpp::Time ot_abort_time_{0, 0, RCL_ROS_TIME};
  double init_sigma_xy_ = 0.3, init_sigma_yaw_ = 0.25, reset_sigma_xy_ = 0.5, reset_sigma_yaw_ = 0.35;
  double loc_score_good_ = 0.55, loc_score_lost_ = 0.3, loc_max_spread_ = 0.25, loc_max_lateral_ = 0.8;
  double trail_gap_ = 0.8, trail_time_gap_ = 0.25, trail_kp_ = 1.2, trail_min_gap_ = 0.35;
  double trail_lane_ = 0.45, overtake_ratio_ = 0.8, overtake_cap_ = 2.5, overtake_after_ = 1.0;
  int deadman_button_ = 5, reset_button_ = 0, pf_beams_ = 108, pf_global_particles_ = 20000;
  int loc_good_scans_ = 12, loc_lost_scans_ = 8;
  Pose2D laser_, init_pose_;

  // state
  State state_ = State::kReactive;
  std::deque<OdomSample> odom_;
  rclcpp::Time last_scan_time_, last_odom_wall_{0, 0, RCL_ROS_TIME}, last_joy_{0, 0, RCL_ROS_TIME};
  rclcpp::Time aeb_until_{0, 0, RCL_ROS_TIME}, trail_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};
  std::vector<float> pf_eps_, aeb_pts_, map_pts_, foreign_pts_;
  Pose2D map_odom_, pf_odom_;
  bool have_map_odom_ = false, have_pf_odom_ = false, localized_ = false, global_mode_ = false;
  bool direction_checked_ = false, deadman_ = false, reset_pressed_ = false, have_last_s_ = false;
  int good_count_ = 0, bad_count_ = 0, line_hint_ = -1, lap_count_ = 0, aeb_events_ = 0;
  double last_steer_cmd_ = 0.0, last_speed_cmd_ = 0.0, ttc_ = 1e9, ego_s_ = 0.0, v_ref_ = 0.0;
  double last_s_ = 0.0, lap_start_ = -1.0, last_lap_ = 0.0, opp_gap_ = 1e9;
  double lookahead_[2] = {0.0, 0.0};
  rclcpp::Time last_dets_time_{0, 0, RCL_ROS_TIME};
  std::vector<std::array<float, 2>> cam_bearings_;
  bool opp_confirmed_ = false;
  std::vector<double> latencies_, pf_latencies_;
  std::vector<std::array<double, 3>> sectors_;
};

}  // namespace race_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<race_core::RaceCoreNode>());
  rclcpp::shutdown();
  return 0;
}
