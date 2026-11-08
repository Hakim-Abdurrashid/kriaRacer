// dpu_detector: camera frames -> YOLOv8n-ReLU on the DPU -> car detections with bearings.
//
// Subscribes the RealSense color stream, runs YoloDpu (letterbox on the CPU, network on the DPU,
// DFL decode + NMS on the CPU) and publishes
//   /perception/camera_dets   std_msgs/Float32MultiArray
//       [latency_ms, n, then per detection: x1, y1, x2, y2, score, bearing, bearing_left, bearing_right]
//       latency_ms = image stamp -> publish; bearings in rad in base_link convention (left positive),
//       including the camera mount yaw.
//   /perception/debug_image   sensor_msgs/Image (bgr8, boxes drawn), at debug_rate Hz, only when watched
// Racing never depends on this node: race_core falls back to LiDAR-only perception without it.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "dpu_detector/yolo_dpu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class DpuDetectorNode : public rclcpp::Node
{
public:
  DpuDetectorNode()
  : Node("dpu_detector")
  {
    const auto model = declare_parameter("model", std::string(""));
    det_.conf = static_cast<float>(declare_parameter("conf", 0.35));
    det_.iou = static_cast<float>(declare_parameter("iou", 0.6));
    det_.max_w_frac = static_cast<float>(declare_parameter("max_box_width_frac", 0.6));
    det_.max_h_frac = static_cast<float>(declare_parameter("max_box_height_frac", 0.6));
    debug_rate_ = declare_parameter("debug_rate", 2.0);
    camera_yaw_ = declare_parameter("camera_yaw", 0.0);
    fx_ = declare_parameter("fx", 615.0);    // overwritten by camera_info when it arrives
    cx_ = declare_parameter("cx", 320.0);
    std::string err;
    if (!det_.load(model, &err)) {
      throw std::runtime_error("dpu_detector: " + err);
    }
    dets_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/perception/camera_dets", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/perception/debug_image", 2);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter("image_topic", std::string("/camera/camera/color/image_raw")),
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onImage(*m);});
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      declare_parameter("camera_info_topic", std::string("/camera/camera/color/camera_info")),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        if (m->k[0] > 0.0 && !have_info_) {
          fx_ = m->k[0];
          cx_ = m->k[2];
          have_info_ = true;
          RCLCPP_INFO(get_logger(), "camera intrinsics: fx %.1f cx %.1f", fx_, cx_);
        }
      });
    RCLCPP_INFO(get_logger(), "dpu_detector up: %s (input %dx%d), conf %.2f", model.c_str(),
      det_.inputWidth(), det_.inputHeight(), det_.conf);
  }

private:
  double bearing(double u) const {return camera_yaw_ - std::atan((u - cx_) / fx_);}

  void onImage(const sensor_msgs::msg::Image & m)
  {
    const auto t0 = std::chrono::steady_clock::now();
    bool rgb;
    if (m.encoding == "rgb8") {rgb = true;} else if (m.encoding == "bgr8") {rgb = false;} else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported encoding %s", m.encoding.c_str());
      return;
    }
    const cv::Mat img(m.height, m.width, CV_8UC3, const_cast<uint8_t *>(m.data.data()), m.step);
    const auto dets = det_.detect(img, rgb);

    std_msgs::msg::Float32MultiArray out;
    const double latency = (now() - rclcpp::Time(m.header.stamp)).seconds() * 1e3;
    out.data.reserve(2 + 8 * dets.size());
    out.data.push_back(static_cast<float>(latency));
    out.data.push_back(static_cast<float>(dets.size()));
    for (const auto & d : dets) {
      const float vals[8] = {d.x1, d.y1, d.x2, d.y2, d.score,
        static_cast<float>(bearing(0.5 * (d.x1 + d.x2))), static_cast<float>(bearing(d.x1)),
        static_cast<float>(bearing(d.x2))};
      out.data.insert(out.data.end(), vals, vals + 8);
    }
    dets_pub_->publish(out);

    if (debug_pub_->get_subscription_count() > 0 && debug_rate_ > 0 &&
      (now() - last_debug_).seconds() >= 1.0 / debug_rate_)
    {
      last_debug_ = now();
      cv::Mat vis;
      if (rgb) {cv::cvtColor(img, vis, cv::COLOR_RGB2BGR);} else {vis = img.clone();}
      for (const auto & d : dets) {
        cv::rectangle(vis, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2), cv::Scalar(0, 255, 0), 2);
        cv::putText(vis, cv::format("car %.2f %+.0fdeg", d.score, bearing(0.5 * (d.x1 + d.x2)) * 57.3),
          cv::Point(d.x1, std::max(12.0f, d.y1 - 4)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          cv::Scalar(0, 255, 0), 1);
      }
      sensor_msgs::msg::Image dbg;
      dbg.header = m.header;
      dbg.height = vis.rows;
      dbg.width = vis.cols;
      dbg.encoding = "bgr8";
      dbg.step = vis.cols * 3;
      dbg.data.assign(vis.data, vis.data + vis.total() * 3);
      debug_pub_->publish(dbg);
    }

    total_ms_.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    dpu_ms_.push_back(det_.t_dpu);
    if (total_ms_.size() >= 150) {
      std::sort(total_ms_.begin(), total_ms_.end());
      std::sort(dpu_ms_.begin(), dpu_ms_.end());
      RCLCPP_INFO(get_logger(), "frame: total p50 %.1f ms p99 %.1f ms (DPU p50 %.1f ms), last latency %.0f ms, "
        "%zu dets", total_ms_[total_ms_.size() / 2], total_ms_[total_ms_.size() * 99 / 100],
        dpu_ms_[dpu_ms_.size() / 2], latency, dets.size());
      total_ms_.clear();
      dpu_ms_.clear();
    }
  }

  dpu_detector::YoloDpu det_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dets_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Time last_debug_{0, 0, RCL_ROS_TIME};
  double debug_rate_, camera_yaw_, fx_, cx_;
  bool have_info_ = false;
  std::vector<double> total_ms_, dpu_ms_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DpuDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
