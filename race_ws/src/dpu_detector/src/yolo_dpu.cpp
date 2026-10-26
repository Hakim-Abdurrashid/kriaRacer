#include "dpu_detector/yolo_dpu.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <vart/runner_ext.hpp>
#include <vart/tensor_buffer.hpp>
#include <xir/graph/graph.hpp>

namespace dpu_detector
{

namespace
{
double msSince(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

float iouOf(const Detection & a, const Detection & b)
{
  const float x1 = std::max(a.x1, b.x1), y1 = std::max(a.y1, b.y1);
  const float x2 = std::min(a.x2, b.x2), y2 = std::min(a.y2, b.y2);
  const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float ua = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return inter / std::max(ua, 1e-6f);
}
}  // namespace

YoloDpu::YoloDpu() = default;
YoloDpu::~YoloDpu() = default;

bool YoloDpu::load(const std::string & xmodel, std::string * error)
{
  try {
    graph_ = xir::Graph::deserialize(xmodel);
  } catch (const std::exception & e) {
    if (error) {*error = std::string("cannot read ") + xmodel + ": " + e.what();}
    return false;
  }
  const xir::Subgraph * dpu = nullptr;
  int n_dpu = 0;
  for (auto * s : graph_->get_root_subgraph()->children_topological_sort()) {
    if (s->has_attr("device") && s->get_attr<std::string>("device") == "DPU") {
      dpu = s;
      ++n_dpu;
    }
  }
  if (!dpu || n_dpu != 1) {
    if (error) {*error = "expected exactly one DPU subgraph, found " + std::to_string(n_dpu);}
    return false;
  }
  attrs_ = xir::Attrs::create();
  runner_ = vart::RunnerExt::create_runner(dpu, attrs_.get());
  in_ = runner_->get_inputs();
  out_ = runner_->get_outputs();
  const auto * it = in_.at(0)->get_tensor();
  const auto shape = it->get_shape();          // N H W C
  in_h_ = shape.at(1);
  in_w_ = shape.at(2);
  const float in_scale = std::exp2(static_cast<float>(it->get_attr<int>("fix_point")));
  for (int v = 0; v < 256; ++v) {
    lut_[v] = static_cast<int8_t>(std::clamp<long>(std::lround(v * in_scale / 255.0f), -128, 127));
  }
  // pair box (C=64) and class (C=1) outputs by spatial size; stride from the input size
  std::vector<int> boxes, clss;
  for (int i = 0; i < static_cast<int>(out_.size()); ++i) {
    const int c = out_[i]->get_tensor()->get_shape().back();
    (c == 64 ? boxes : clss).push_back(i);
  }
  if (boxes.size() != 3 || clss.size() != 3) {
    if (error) {*error = "unexpected outputs: need 3 box (C=64) and 3 class (C=1) maps";}
    return false;
  }
  for (int b : boxes) {
    Head h;
    h.box = b;
    const auto bs = out_[b]->get_tensor()->get_shape();
    h.h = bs.at(1);
    h.w = bs.at(2);
    h.stride = in_h_ / h.h;
    for (int c : clss) {
      const auto cs = out_[c]->get_tensor()->get_shape();
      if (cs.at(1) == h.h && cs.at(2) == h.w) {h.cls = c;}
    }
    h.box_scale = std::exp2(-static_cast<float>(out_[b]->get_tensor()->get_attr<int>("fix_point")));
    h.cls_scale = std::exp2(-static_cast<float>(out_[h.cls]->get_tensor()->get_attr<int>("fix_point")));
    heads_.push_back(h);
  }
  return true;
}

std::vector<Detection> YoloDpu::detect(const cv::Mat & img, bool rgb)
{
  // 1. letterbox into the int8 input buffer (pad 114 like ultralytics)
  auto t0 = std::chrono::steady_clock::now();
  const float s = std::min(static_cast<float>(in_w_) / img.cols, static_cast<float>(in_h_) / img.rows);
  const int rw = static_cast<int>(std::round(img.cols * s)), rh = static_cast<int>(std::round(img.rows * s));
  const int left = (in_w_ - rw) / 2, top = (in_h_ - rh) / 2;
  cv::resize(img, resized_, cv::Size(rw, rh), 0, 0, cv::INTER_LINEAR);
  uint64_t addr = 0;
  size_t size = 0;
  std::tie(addr, size) = in_[0]->data({0, 0, 0, 0});
  int8_t * dst = reinterpret_cast<int8_t *>(addr);
  const int8_t pad = lut_[114];
  std::fill(dst, dst + static_cast<size_t>(in_h_) * in_w_ * 3, pad);
  const int r0 = rgb ? 0 : 2, r2 = rgb ? 2 : 0;   // model wants RGB
  for (int y = 0; y < rh; ++y) {
    const uint8_t * src = resized_.ptr<uint8_t>(y);
    int8_t * row = dst + (static_cast<size_t>(y + top) * in_w_ + left) * 3;
    for (int x = 0; x < rw; ++x) {
      row[3 * x + 0] = lut_[src[3 * x + r0]];
      row[3 * x + 1] = lut_[src[3 * x + 1]];
      row[3 * x + 2] = lut_[src[3 * x + r2]];
    }
  }
  in_[0]->sync_for_write(0, size);
  t_pre = msSince(t0);

  // 2. DPU
  t0 = std::chrono::steady_clock::now();
  const auto job = runner_->execute_async(in_, out_);
  runner_->wait(static_cast<int>(job.first), -1);
  t_dpu = msSince(t0);

  // 3. decode candidates above the confidence threshold (compare in the int8 domain first)
  t0 = std::chrono::steady_clock::now();
  std::vector<Detection> cand;
  const float logit_th = std::log(conf / (1.0f - conf));
  for (const auto & h : heads_) {
    uint64_t ca = 0, ba = 0;
    size_t cs = 0, bsz = 0;
    std::tie(ca, cs) = out_[h.cls]->data({0, 0, 0, 0});
    std::tie(ba, bsz) = out_[h.box]->data({0, 0, 0, 0});
    out_[h.cls]->sync_for_read(0, cs);
    const int8_t * cls = reinterpret_cast<const int8_t *>(ca);
    const int8_t * box = reinterpret_cast<const int8_t *>(ba);
    const int th = static_cast<int>(std::floor(logit_th / h.cls_scale));
    bool synced_box = false;
    for (int i = 0; i < h.h * h.w; ++i) {
      if (cls[i] <= th) {continue;}
      const float logit = cls[i] * h.cls_scale;
      const float score = 1.0f / (1.0f + std::exp(-logit));
      if (score <= conf) {continue;}
      if (!synced_box) {
        out_[h.box]->sync_for_read(0, bsz);
        synced_box = true;
      }
      float ltrb[4];
      const int8_t * b = box + static_cast<size_t>(i) * 64;
      for (int side = 0; side < 4; ++side) {   // DFL: softmax over 16 bins, expectation
        float mx = -1e9f;
        for (int k = 0; k < 16; ++k) {mx = std::max(mx, b[side * 16 + k] * h.box_scale);}
        float sum = 0.0f, acc = 0.0f;
        for (int k = 0; k < 16; ++k) {
          const float e = std::exp(b[side * 16 + k] * h.box_scale - mx);
          sum += e;
          acc += e * k;
        }
        ltrb[side] = acc / sum;
      }
      const float cx = (i % h.w) + 0.5f, cy = (i / h.w) + 0.5f;
      Detection d;
      d.x1 = ((cx - ltrb[0]) * h.stride - left) / s;
      d.y1 = ((cy - ltrb[1]) * h.stride - top) / s;
      d.x2 = ((cx + ltrb[2]) * h.stride - left) / s;
      d.y2 = ((cy + ltrb[3]) * h.stride - top) / s;
      d.score = score;
      const float bw = d.x2 - d.x1, bh = d.y2 - d.y1;
      if (bw > max_w_frac * img.cols || bh > max_h_frac * img.rows || bw < min_size_px ||
        bh < min_size_px) {continue;}
      cand.push_back(d);
    }
  }
  // 4. greedy NMS
  std::sort(cand.begin(), cand.end(), [](const Detection & a, const Detection & b) {return a.score > b.score;});
  std::vector<Detection> out;
  for (const auto & d : cand) {
    bool keep = true;
    for (const auto & k : out) {
      if (iouOf(d, k) > iou) {
        keep = false;
        break;
      }
    }
    if (keep) {
      out.push_back(d);
      if (static_cast<int>(out.size()) >= max_det) {break;}
    }
  }
  for (auto & d : out) {   // clip to the image
    d.x1 = std::clamp(d.x1, 0.0f, static_cast<float>(img.cols));
    d.x2 = std::clamp(d.x2, 0.0f, static_cast<float>(img.cols));
    d.y1 = std::clamp(d.y1, 0.0f, static_cast<float>(img.rows));
    d.y2 = std::clamp(d.y2, 0.0f, static_cast<float>(img.rows));
  }
  t_post = msSince(t0);
  return out;
}

}  // namespace dpu_detector
