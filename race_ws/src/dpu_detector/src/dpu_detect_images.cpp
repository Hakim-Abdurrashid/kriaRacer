// Offline check of the DPU detector on image files (no ROS).
//
// usage: dpu_detect_images MODEL.xmodel IMAGE_DIR [--labels LABEL_DIR] [--conf 0.25] [--draw OUT_DIR]
//                          [--csv OUT.csv]
// With YOLO-format labels it prints precision/recall at IoU 0.5 (same metric as models/dpu_check.py);
// without labels every detection is listed, which on frames with no cars is the false-positive count.
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "dpu_detector/yolo_dpu.hpp"

namespace fs = std::filesystem;
using dpu_detector::Detection;

static float iouOf(const Detection & a, const Detection & b)
{
  const float x1 = std::max(a.x1, b.x1), y1 = std::max(a.y1, b.y1);
  const float x2 = std::min(a.x2, b.x2), y2 = std::min(a.y2, b.y2);
  const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  return inter / ((a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter + 1e-9f);
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s MODEL.xmodel IMAGE_DIR [--labels DIR] [--conf C] [--draw DIR] [--csv F]\n",
      argv[0]);
    return 2;
  }
  std::string model = argv[1], images = argv[2], labels, draw, csv;
  float conf = 0.25f, max_frac = 0.6f;
  for (int i = 3; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--labels") {labels = argv[i + 1];}
    else if (k == "--conf") {conf = std::stof(argv[i + 1]);}
    else if (k == "--draw") {draw = argv[i + 1];}
    else if (k == "--csv") {csv = argv[i + 1];}
    else if (k == "--max-frac") {max_frac = std::stof(argv[i + 1]);}
  }
  dpu_detector::YoloDpu det;
  std::string err;
  if (!det.load(model, &err)) {
    std::fprintf(stderr, "load failed: %s\n", err.c_str());
    return 1;
  }
  det.conf = conf;
  det.max_w_frac = det.max_h_frac = max_frac;   // 1.0 disables the geometry filter
  std::printf("model input %dx%d, conf %.2f\n", det.inputWidth(), det.inputHeight(), conf);
  std::vector<fs::path> files;
  for (const auto & e : fs::directory_iterator(images)) {
    const auto ext = e.path().extension().string();
    if (ext == ".jpg" || ext == ".png" || ext == ".jpeg") {files.push_back(e.path());}
  }
  std::sort(files.begin(), files.end());
  if (!draw.empty()) {fs::create_directories(draw);}
  std::ofstream out;
  if (!csv.empty()) {
    out.open(csv);
    out << "file,x1,y1,x2,y2,score\n";
  }
  int tp = 0, fp = 0, ngt = 0, frames_with_dets = 0;
  std::vector<double> pre, dpu, post;
  for (const auto & f : files) {
    cv::Mat img = cv::imread(f.string(), cv::IMREAD_COLOR);
    if (img.empty()) {continue;}
    const auto dets = det.detect(img, false);
    pre.push_back(det.t_pre);
    dpu.push_back(det.t_dpu);
    post.push_back(det.t_post);
    if (!dets.empty()) {++frames_with_dets;}
    for (const auto & d : dets) {
      if (out) {out << f.filename().string() << ',' << d.x1 << ',' << d.y1 << ',' << d.x2 << ',' << d.y2 << ',' << d.score << '\n';}
    }
    if (!labels.empty()) {
      std::vector<Detection> gt;
      std::ifstream lf((fs::path(labels) / f.stem()).string() + ".txt");
      int c;
      float cx, cy, w, h;
      while (lf >> c >> cx >> cy >> w >> h) {
        gt.push_back({(cx - w / 2) * img.cols, (cy - h / 2) * img.rows, (cx + w / 2) * img.cols,
            (cy + h / 2) * img.rows, 1.0f});
      }
      ngt += static_cast<int>(gt.size());
      std::vector<bool> used(gt.size(), false);
      for (const auto & d : dets) {
        float best = 0;
        int bj = -1;
        for (size_t j = 0; j < gt.size(); ++j) {
          if (used[j]) {continue;}
          const float v = iouOf(d, gt[j]);
          if (v > best) {
            best = v;
            bj = static_cast<int>(j);
          }
        }
        if (best >= 0.5f) {
          ++tp;
          used[bj] = true;
        } else {
          ++fp;
        }
      }
    }
    if (!draw.empty() && !dets.empty()) {
      for (const auto & d : dets) {
        cv::rectangle(img, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2), cv::Scalar(0, 255, 0), 2);
        cv::putText(img, cv::format("%.2f", d.score), cv::Point(d.x1, std::max(12.0f, d.y1 - 3)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      }
      cv::imwrite((fs::path(draw) / f.filename()).string(), img);
    }
  }
  auto p = [](std::vector<double> v, double q) {
      if (v.empty()) {return 0.0;}
      std::sort(v.begin(), v.end());
      return v[std::min(v.size() - 1, static_cast<size_t>(q * v.size()))];
    };
  std::printf("%zu images, %d with detections\n", files.size(), frames_with_dets);
  if (!labels.empty()) {
    std::printf("%d labelled cars: TP %d, FP %d, missed %d -> precision %.3f, recall %.3f (IoU 0.5)\n", ngt,
      tp, fp, ngt - tp, tp / std::max(1.0, double(tp + fp)), tp / std::max(1.0, double(ngt)));
  }
  std::printf("per frame (C++): preprocess p50 %.2f ms, DPU p50 %.2f p99 %.2f ms, decode+NMS p50 %.2f ms\n",
    p(pre, 0.5), p(dpu, 0.5), p(dpu, 0.99), p(post, 0.5));
  return 0;
}
