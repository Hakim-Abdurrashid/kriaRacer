// YOLOv8 (ReLU, single class) on the Kria DPU through VART, with CPU letterbox + DFL decode + NMS.
//
// The xmodel comes from ml/quantize_dpu.py: one DPU subgraph, int8 NHWC input (RGB / 255 scaled by
// 2^fix_point), six int8 outputs = per stride (8, 16, 32) a box map (H, W, 64 = 4 sides x 16 DFL bins)
// and a class-logit map (H, W, 1). Reference implementation: models/dpu_check.py.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace xir
{
class Graph;
class Attrs;
}
namespace vart
{
class RunnerExt;
class TensorBuffer;
}

namespace dpu_detector
{

struct Detection
{
  float x1, y1, x2, y2;   // pixels in the input image
  float score;
};

class YoloDpu
{
public:
  YoloDpu();
  ~YoloDpu();
  bool load(const std::string & xmodel, std::string * error);
  // img: 8UC3, any size; rgb=false means OpenCV BGR order.
  std::vector<Detection> detect(const cv::Mat & img, bool rgb);

  float conf = 0.25f;
  float iou = 0.6f;
  int max_det = 20;
  // Geometry sanity limits (fractions of the image / pixels). An opponent close enough to fill most
  // of the frame would already be touching us; frame-filling boxes are the typical false positive on
  // our own D435i frames (blurred ducts and walls, July bags).
  float max_w_frac = 0.6f;
  float max_h_frac = 0.6f;
  float min_size_px = 6.0f;
  // timings of the last detect() call, ms
  double t_pre = 0, t_dpu = 0, t_post = 0;
  int inputWidth() const {return in_w_;}
  int inputHeight() const {return in_h_;}

private:
  struct Head
  {
    int box = -1, cls = -1, h = 0, w = 0, stride = 0;
    float box_scale = 1, cls_scale = 1;
  };
  std::unique_ptr<xir::Graph> graph_;
  std::unique_ptr<xir::Attrs> attrs_;
  std::unique_ptr<vart::RunnerExt> runner_;
  std::vector<vart::TensorBuffer *> in_, out_;
  std::vector<Head> heads_;
  int in_h_ = 0, in_w_ = 0;
  int8_t lut_[256];
  cv::Mat resized_;
};

}  // namespace dpu_detector
