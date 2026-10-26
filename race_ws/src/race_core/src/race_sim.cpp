// race_sim: minimal closed-loop simulator for race_core on a map_server map (RACE_PLAN.md §9).
//
// Kinematic bicycle with actuator dynamics (speed lag + accel/brake limits, steering rate limit,
// command delay), a lateral-friction limit (the car runs wide when v^2 * kappa > mu * g), collision
// checks against the map's occupied cells, a Hokuyo-like LiDAR ray-marched on the distance field,
// a drifting odometry stream (stands in for the EKF), and an optional opponent that drives the
// raceline at a fixed fraction of its speed profile.
//
// Publishes /scan (40 Hz), /odometry/filtered + TF odom->base_link (100 Hz), /sim/pose (truth),
// /sim/opponent, /sim/status [crashes, laps, last_lap_s, opp_crashes, t]. Subscribes /drive.
// A crash stops the car; after crash_reset_s it is put back on the raceline (like a hand reset)
// and a /joy message with the reset button pressed is sent so race_core re-localizes.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "race_core/geometry.hpp"
#include "race_core/occupancy_map.hpp"
#include "race_core/raceline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace race_core
{

geometry_msgs::msg::Quaternion simYawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

class RaceSim : public rclcpp::Node
{
public:
  RaceSim()
  : Node("race_sim"), rng_(7)
  {
    const auto map_yaml = declare_parameter("map_yaml", std::string(""));
    const auto line_csv = declare_parameter("raceline_csv", std::string(""));
    rate_ = declare_parameter("physics_rate", 200.0);
    wb_ = declare_parameter("wheelbase", 0.33);
    mu_ = declare_parameter("mu", 0.6);
    a_max_ = declare_parameter("accel_max", 4.0);
    b_max_ = declare_parameter("brake_max", 6.0);
    tau_v_ = declare_parameter("speed_tau", 0.15);
    steer_rate_ = declare_parameter("steer_rate", 4.0);
    max_steer_ = declare_parameter("max_steer", 0.42);
    delay_ = declare_parameter("command_delay", 0.04);
    beams_ = declare_parameter("lidar.beams", 1081);
    fov_ = declare_parameter("lidar.fov", 4.712389);
    range_max_ = declare_parameter("lidar.range_max", 10.0);
    range_noise_ = declare_parameter("lidar.noise", 0.01);
    lidar_rate_ = declare_parameter("lidar.rate", 40.0);
    // >0: each beam is cast from the pose at its own time (rotating-mirror skew, like the Hokuyo:
    // 25 ms per revolution / 1440 steps); the stamp is the first beam's time, as urg_node does
    time_inc_ = declare_parameter("lidar.time_increment", 0.025 / 1440.0);
    laser_x_ = declare_parameter("laser_x", 0.27);
    odom_scale_err_ = declare_parameter("odom.speed_scale_error", 0.02);
    odom_speed_noise_ = declare_parameter("odom.speed_noise", 0.05);
    gyro_bias_ = declare_parameter("odom.gyro_bias", 0.01);
    gyro_noise_ = declare_parameter("odom.gyro_noise", 0.02);
    car_front_ = declare_parameter("car.front", 0.42);   // from base_link (rear axle)
    car_rear_ = declare_parameter("car.rear", 0.08);
    car_half_w_ = declare_parameter("car.half_width", 0.15);
    opp_enable_ = declare_parameter("opponent.enable", false);
    opp_fraction_ = declare_parameter("opponent.speed_fraction", 0.5);
    opp_gap_ = declare_parameter("opponent.start_gap", 3.0);
    opp_offset_ = declare_parameter("opponent.lateral_offset", 0.0);
    crash_reset_s_ = declare_parameter("crash_reset_s", 1.0);
    reset_button_ = declare_parameter("reset_button", 0);

    std::string err;
    if (!map_.load(map_yaml, &err)) {
      throw std::runtime_error("race_sim: " + err);
    }
    if (!line_csv.empty() && !line_.load(line_csv, &err)) {
      throw std::runtime_error("race_sim: " + err);
    }
    Pose2D start{declare_parameter("start_x", 0.0), declare_parameter("start_y", 0.0),
      declare_parameter("start_yaw", 0.0)};
    if (declare_parameter("start_on_raceline", true) && !line_.empty()) {
      const auto p = line_[0];
      start = {p.x, p.y, p.psi};
    }
    car_ = start;
    odom_pose_ = Pose2D{};
    if (opp_enable_ && !line_.empty()) {opp_s_ = line_.wrapS(opp_gap_);}

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 20);
    vesc_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);   // like vesc_to_odom
    truth_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim/pose", 10);
    opp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim/opponent", 10);
    status_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/sim/status", 10);
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10,
        [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
          cmds_.push_back({now().seconds() + delay_, m->drive.speed, m->drive.steering_angle});
        });
    const auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {step();});
    last_t_ = now().seconds();
    RCLCPP_INFO(get_logger(), "race_sim: map %dx%d, start (%.2f, %.2f, %.0f deg), opponent %s",
      map_.width(), map_.height(), car_.x, car_.y, car_.yaw * 180 / M_PI, opp_enable_ ? "ON" : "off");
  }

private:
  struct Cmd
  {
    double t, speed, steer;
  };

  void step()
  {
    const double t = now().seconds();
    double dt = t - last_t_;
    last_t_ = t;
    if (dt <= 0.0 || dt > 0.05) {dt = 1.0 / rate_;}
    while (!cmds_.empty() && cmds_.front().t <= t) {
      cmd_speed_ = cmds_.front().speed;
      cmd_steer_ = cmds_.front().steer;
      cmds_.pop_front();
    }
    std::normal_distribution<double> n01(0.0, 1.0);

    if (crashed_) {
      v_ = 0.0;
      if (t - crash_t_ > crash_reset_s_) {handReset();}
    } else {
      // actuators
      const double acc = std::clamp((cmd_speed_ - v_) / tau_v_, -b_max_, a_max_);
      v_ += acc * dt;
      const double target = std::clamp(cmd_steer_, -max_steer_, max_steer_);
      steer_ += std::clamp(target - steer_, -steer_rate_ * dt, steer_rate_ * dt);
      // kinematics with a friction limit on lateral acceleration
      double kappa = std::tan(steer_) / wb_;
      const double alat_max = mu_ * 9.81;
      if (v_ * v_ * std::fabs(kappa) > alat_max) {
        kappa = std::copysign(alat_max / (v_ * v_), kappa);
        ++slide_steps_;
      }
      const double w = v_ * kappa;
      car_.x += v_ * std::cos(car_.yaw) * dt;
      car_.y += v_ * std::sin(car_.yaw) * dt;
      car_.yaw = normalizeAngle(car_.yaw + w * dt);
      // odometry as the EKF would see it: scaled/noisy speed, biased/noisy gyro
      const double v_meas = v_ * (1.0 + odom_scale_err_) + odom_speed_noise_ * n01(rng_);
      const double w_meas = w + gyro_bias_ + gyro_noise_ * n01(rng_);
      odom_pose_.x += v_meas * std::cos(odom_pose_.yaw) * dt;
      odom_pose_.y += v_meas * std::sin(odom_pose_.yaw) * dt;
      odom_pose_.yaw = normalizeAngle(odom_pose_.yaw + w_meas * dt);
      odom_v_ = v_meas;
      odom_w_ = w_meas;
      if (collides(car_)) {
        crash("wall");
      }
    }
    // opponent
    if (opp_enable_ && !line_.empty()) {
      if (!opp_crashed_) {
        opp_s_ = line_.wrapS(opp_s_ + opp_fraction_ * line_.at(opp_s_).v * dt);
      } else if (t - opp_crash_t_ > crash_reset_s_) {
        opp_crashed_ = false;
      }
      if (!crashed_ && carsOverlap()) {
        ++opp_crashes_;
        opp_crashed_ = true;
        opp_crash_t_ = t;
        crash("opponent");
      }
    }
    lapCheck(t);

    if (t - last_odom_pub_ >= 0.01 - 1e-4) {
      last_odom_pub_ = t;
      publishOdom();
    }
    if (t - last_scan_pub_ >= 1.0 / lidar_rate_ - 1e-4) {
      last_scan_pub_ = t;
      publishScan();
    }
    if (t - last_status_pub_ >= 0.05) {
      last_status_pub_ = t;
      publishTruth();
    }
  }

  Pose2D oppPose() const
  {
    const auto p = line_.at(opp_s_);
    return {p.x - std::sin(p.psi) * opp_offset_, p.y + std::cos(p.psi) * opp_offset_, p.psi};
  }

  bool collides(const Pose2D & c) const
  {
    const double xs[] = {-car_rear_, 0.5 * (car_front_ - car_rear_), car_front_};
    const double ys[] = {-car_half_w_, 0.0, car_half_w_};
    const double cc = std::cos(c.yaw), ss = std::sin(c.yaw);
    for (double x : xs) {
      for (double y : ys) {
        if (x == 0.5 * (car_front_ - car_rear_) && y == 0.0) {continue;}
        const double wx = c.x + cc * x - ss * y, wy = c.y + ss * x + cc * y;
        if (map_.cellAt(wx, wy) == OccupancyMap::kOccupied) {return true;}
      }
    }
    return false;
  }

  bool carsOverlap() const
  {
    // corners of each car tested inside the other (cars are small, this is enough)
    const Pose2D o = oppPose();
    auto inside = [&](const Pose2D & a, const Pose2D & b) {   // any corner of a inside b
        const double ca = std::cos(a.yaw), sa = std::sin(a.yaw);
        const double cb = std::cos(b.yaw), sb = std::sin(b.yaw);
        for (double x : {-car_rear_, car_front_}) {
          for (double y : {-car_half_w_, car_half_w_}) {
            const double wx = a.x + ca * x - sa * y, wy = a.y + sa * x + ca * y;
            const double lx = cb * (wx - b.x) + sb * (wy - b.y);
            const double ly = -sb * (wx - b.x) + cb * (wy - b.y);
            if (lx > -car_rear_ && lx < car_front_ && std::fabs(ly) < car_half_w_) {return true;}
          }
        }
        return false;
      };
    return inside(car_, o) || inside(o, car_);
  }

  void crash(const char * what)
  {
    crashed_ = true;
    crash_t_ = now().seconds();
    ++crashes_;
    v_ = 0.0;
    RCLCPP_WARN(get_logger(), "CRASH #%d (%s) at (%.2f, %.2f)", crashes_, what, car_.x, car_.y);
  }

  void handReset()
  {
    crashed_ = false;
    if (!line_.empty()) {
      // put the car down on the line, 0.3 m back, facing the way it was driving
      const auto pr = line_.project(car_.x, car_.y);
      const bool fwd = std::fabs(normalizeAngle(car_.yaw - pr.psi)) < M_PI / 2;
      const auto p = line_.at(pr.s + (fwd ? -0.3 : 0.3));
      car_ = {p.x, p.y, fwd ? p.psi : normalizeAngle(p.psi + M_PI)};
    }
    steer_ = 0.0;
    cmds_.clear();
    cmd_speed_ = 0.0;
    RCLCPP_WARN(get_logger(), "hand reset to (%.2f, %.2f); pressing RESET", car_.x, car_.y);
    sensor_msgs::msg::Joy j;
    j.header.stamp = now();
    j.buttons.assign(std::max(8, reset_button_ + 1), 0);
    j.buttons[reset_button_] = 1;
    joy_pub_->publish(j);
    j.buttons[reset_button_] = 0;
    joy_pub_->publish(j);
  }

  void lapCheck(double t)
  {
    if (line_.empty()) {return;}
    const auto pr = line_.project(car_.x, car_.y, hint_, 60);
    hint_ = pr.index;
    const double L = line_.length();
    const bool fwd = last_s_ > 0.75 * L && pr.s < 0.25 * L;   // either driving direction
    const bool back = last_s_ < 0.25 * L && pr.s > 0.75 * L;
    if (have_s_ && (fwd || back)) {
      if (lap_start_ > 0) {
        last_lap_ = t - lap_start_;
        ++laps_;
        RCLCPP_INFO(get_logger(), "SIM LAP %d: %.2f s (crashes so far %d, slide steps %d)", laps_,
          last_lap_, crashes_, slide_steps_);
      }
      lap_start_ = t;
    }
    last_s_ = pr.s;
    have_s_ = true;
  }

  double rayMap(double x, double y, double dx, double dy) const
  {
    const double res = map_.resolution();
    double t = 0.0;
    while (t < range_max_) {
      const double px = x + dx * t, py = y + dy * t;
      int cx, cy;
      if (!map_.worldToCell(px, py, cx, cy)) {return range_max_ + 1.0;}
      if (map_.cell(cx, cy) == OccupancyMap::kOccupied) {return t;}
      const double d = map_.distanceField()[cy * map_.width() + cx];
      t += std::max(d - 0.7 * res, 0.5 * res);
    }
    return range_max_ + 1.0;
  }

  // ray vs the opponent's rectangle (slab test in the opponent frame)
  double rayOpp(double x, double y, double dx, double dy, const Pose2D & o) const
  {
    const double c = std::cos(o.yaw), s = std::sin(o.yaw);
    const double lx = c * (x - o.x) + s * (y - o.y), ly = -s * (x - o.x) + c * (y - o.y);
    const double ldx = c * dx + s * dy, ldy = -s * dx + c * dy;
    double t0 = -1e9, t1 = 1e9;
    const double lo[2] = {-car_rear_, -car_half_w_}, hi[2] = {car_front_, car_half_w_};
    const double p[2] = {lx, ly}, d[2] = {ldx, ldy};
    for (int k = 0; k < 2; ++k) {
      if (std::fabs(d[k]) < 1e-12) {
        if (p[k] < lo[k] || p[k] > hi[k]) {return 1e9;}
      } else {
        double a = (lo[k] - p[k]) / d[k], b = (hi[k] - p[k]) / d[k];
        if (a > b) {std::swap(a, b);}
        t0 = std::max(t0, a);
        t1 = std::min(t1, b);
      }
    }
    if (t0 > t1 || t1 < 0) {return 1e9;}
    return t0 > 0 ? t0 : 1e9;
  }

  void publishScan()
  {
    sensor_msgs::msg::LaserScan s;
    s.header.stamp = now();
    s.header.frame_id = "laser";
    s.angle_min = static_cast<float>(-0.5 * fov_);
    s.angle_max = static_cast<float>(0.5 * fov_);
    s.angle_increment = static_cast<float>(fov_ / (beams_ - 1));
    s.scan_time = static_cast<float>(1.0 / lidar_rate_);
    s.range_min = 0.02f;
    s.range_max = static_cast<float>(range_max_);
    s.ranges.resize(beams_);
    s.time_increment = static_cast<float>(time_inc_);
    s.header.stamp = now() - rclcpp::Duration::from_seconds(time_inc_ * (beams_ - 1));
    const double w = v_ * std::tan(steer_) / wb_;
    const bool opp = opp_enable_ && !line_.empty();
    const Pose2D o = opp ? oppPose() : Pose2D{};
    std::normal_distribution<double> n01(0.0, 1.0);
    for (int i = 0; i < beams_; ++i) {
      const double dt = (i - (beams_ - 1)) * time_inc_;   // <= 0: this beam was measured earlier
      const double yaw = car_.yaw + w * dt;
      const double cx = car_.x + v_ * dt * std::cos(car_.yaw + 0.5 * w * dt);
      const double cy = car_.y + v_ * dt * std::sin(car_.yaw + 0.5 * w * dt);
      const double lx = cx + laser_x_ * std::cos(yaw);
      const double ly = cy + laser_x_ * std::sin(yaw);
      const double a = yaw + s.angle_min + i * s.angle_increment;
      const double dx = std::cos(a), dy = std::sin(a);
      double r = rayMap(lx, ly, dx, dy);
      if (opp) {r = std::min(r, rayOpp(lx, ly, dx, dy, o));}
      s.ranges[i] = r > range_max_ ? std::numeric_limits<float>::infinity() :
        static_cast<float>(r + range_noise_ * n01(rng_));
    }
    scan_pub_->publish(s);
  }

  void publishOdom()
  {
    nav_msgs::msg::Odometry o;
    o.header.stamp = now();
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = odom_pose_.x;
    o.pose.pose.position.y = odom_pose_.y;
    o.pose.pose.orientation = simYawToQuat(odom_pose_.yaw);
    o.twist.twist.linear.x = odom_v_;
    o.twist.twist.angular.z = odom_w_;
    odom_pub_->publish(o);
    vesc_odom_pub_->publish(o);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = o.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom_pose_.x;
    tf.transform.translation.y = odom_pose_.y;
    tf.transform.rotation = o.pose.pose.orientation;
    tf_pub_->sendTransform(tf);
  }

  void publishTruth()
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now();
    p.header.frame_id = "map";
    p.pose.position.x = car_.x;
    p.pose.position.y = car_.y;
    p.pose.orientation = simYawToQuat(car_.yaw);
    truth_pub_->publish(p);
    if (opp_enable_ && !line_.empty()) {
      const Pose2D o = oppPose();
      p.pose.position.x = o.x;
      p.pose.position.y = o.y;
      p.pose.orientation = simYawToQuat(o.yaw);
      opp_pub_->publish(p);
    }
    std_msgs::msg::Float32MultiArray st;
    st.data = {static_cast<float>(crashes_), static_cast<float>(laps_), static_cast<float>(last_lap_),
      static_cast<float>(opp_crashes_), static_cast<float>(now().seconds()),
      static_cast<float>(v_), static_cast<float>(slide_steps_)};
    status_pub_->publish(st);
  }

  OccupancyMap map_;
  Raceline line_;
  std::mt19937 rng_;
  double rate_, wb_, mu_, a_max_, b_max_, tau_v_, steer_rate_, max_steer_, delay_;
  int beams_;
  double fov_, range_max_, range_noise_, lidar_rate_, laser_x_, time_inc_;
  double odom_scale_err_, odom_speed_noise_, gyro_bias_, gyro_noise_;
  double car_front_, car_rear_, car_half_w_;
  bool opp_enable_;
  double opp_fraction_, opp_gap_, opp_offset_, crash_reset_s_;
  int reset_button_;

  Pose2D car_, odom_pose_;
  double v_ = 0.0, steer_ = 0.0, cmd_speed_ = 0.0, cmd_steer_ = 0.0, odom_v_ = 0.0, odom_w_ = 0.0;
  std::deque<Cmd> cmds_;
  bool crashed_ = false, opp_crashed_ = false, have_s_ = false;
  double crash_t_ = 0.0, opp_crash_t_ = 0.0, opp_s_ = 0.0, last_s_ = 0.0, lap_start_ = -1.0;
  double last_lap_ = 0.0, last_t_ = 0.0, last_odom_pub_ = 0.0, last_scan_pub_ = 0.0;
  double last_status_pub_ = 0.0;
  int crashes_ = 0, opp_crashes_ = 0, laps_ = 0, hint_ = -1, slide_steps_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, vesc_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_, opp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace race_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<race_core::RaceSim>());
  rclcpp::shutdown();
  return 0;
}
