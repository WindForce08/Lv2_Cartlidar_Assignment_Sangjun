// TensorRTDetector — YOLOv8 형식 엔진 (입력 1×3×S×S float, 출력 1×(4+nc)×N: cx, cy, w, h, 클래스 점수)
// TensorRT 10 API (Jetson Orin JetPack 6.x). USE_TENSORRT 일 때만 빌드된다.
//   엔진 만들기 예: trtexec --onnx=puck_yolov8n.onnx --saveEngine=puck_yolov8n.engine --fp16
#ifdef USE_TENSORRT

#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/detector.hpp"

namespace perception
{

namespace
{
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity s, const char * msg) noexcept override
  {
    if (s <= Severity::kWARNING) {std::cerr << "[TensorRT] " << msg << std::endl;}
  }
};

#define CUDA_OK(x) ((x) == cudaSuccess)
}  // namespace

class TensorRTDetector : public Detector
{
public:
  ~TensorRTDetector() override
  {
    if (d_in_) {cudaFree(d_in_);}
    if (d_out_) {cudaFree(d_out_);}
    if (stream_) {cudaStreamDestroy(stream_);}
  }

  bool init(const DetectorConfig & c, std::string & info)
  {
    cfg_ = c;
    std::ifstream f(c.engine_path, std::ios::binary);
    if (!f) {info = "엔진 파일을 열 수 없음: " + c.engine_path; return false;}
    std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {info = "createInferRuntime 실패"; return false;}
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) {info = "엔진 역직렬화 실패 (TensorRT 버전 · GPU 가 엔진을 만든 환경과 같은지 확인)"; return false;}
    ctx_.reset(engine_->createExecutionContext());
    if (!ctx_) {info = "createExecutionContext 실패"; return false;}

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * n = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(n) == nvinfer1::TensorIOMode::kINPUT) {in_name_ = n;} else {out_name_ = n;}
    }
    const auto in_dims = engine_->getTensorShape(in_name_.c_str());
    const auto out_dims = engine_->getTensorShape(out_name_.c_str());
    if (in_dims.nbDims != 4 || out_dims.nbDims != 3) {info = "입출력 모양이 YOLOv8 형식이 아님"; return false;}
    in_h_ = static_cast<int>(in_dims.d[2]);
    in_w_ = static_cast<int>(in_dims.d[3]);
    out_c_ = static_cast<int>(out_dims.d[1]);   // 4 + nc
    out_n_ = static_cast<int>(out_dims.d[2]);   // 후보 수
    if (out_c_ < 5) {info = "출력 채널 수가 5 미만"; return false;}

    const size_t in_bytes = 3ull * in_h_ * in_w_ * sizeof(float);
    const size_t out_bytes = static_cast<size_t>(out_c_) * out_n_ * sizeof(float);
    if (!CUDA_OK(cudaStreamCreate(&stream_)) || !CUDA_OK(cudaMalloc(&d_in_, in_bytes)) ||
      !CUDA_OK(cudaMalloc(&d_out_, out_bytes)))
    {
      info = "CUDA 메모리 할당 실패";
      return false;
    }
    host_out_.resize(static_cast<size_t>(out_c_) * out_n_);
    ctx_->setTensorAddress(in_name_.c_str(), d_in_);
    ctx_->setTensorAddress(out_name_.c_str(), d_out_);
    info = "TensorRT " + c.engine_path + " (입력 " + std::to_string(in_w_) + "×" + std::to_string(in_h_) +
      ", 클래스 " + std::to_string(out_c_ - 4) + ")";
    return true;
  }

  std::vector<Detection> detect(const cv::Mat & bgr) override
  {
    std::vector<Detection> out;
    if (bgr.empty()) {return out;}
    // 레터박스
    const float s = std::min(static_cast<float>(in_w_) / bgr.cols, static_cast<float>(in_h_) / bgr.rows);
    const int nw = static_cast<int>(bgr.cols * s), nh = static_cast<int>(bgr.rows * s);
    const int px = (in_w_ - nw) / 2, py = (in_h_ - nh) / 2;
    cv::Mat resized, canvas(in_h_, in_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::resize(bgr, resized, {nw, nh});
    resized.copyTo(canvas(cv::Rect(px, py, nw, nh)));
    cv::Mat blob = cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);

    cudaMemcpyAsync(d_in_, blob.ptr<float>(), blob.total() * sizeof(float), cudaMemcpyHostToDevice, stream_);
    if (!ctx_->enqueueV3(stream_)) {return out;}
    cudaMemcpyAsync(host_out_.data(), d_out_, host_out_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classes;
    const int nc = out_c_ - 4;
    for (int i = 0; i < out_n_; ++i) {
      int best = -1;
      float bs = 0.f;
      for (int c = 0; c < nc; ++c) {
        const float v = host_out_[static_cast<size_t>(4 + c) * out_n_ + i];
        if (v > bs) {bs = v; best = c;}
      }
      if (bs < cfg_.score_threshold) {continue;}
      if (cfg_.target_class >= 0 && best != cfg_.target_class) {continue;}
      const float cx = host_out_[i], cy = host_out_[out_n_ + i];
      const float w = host_out_[2 * out_n_ + i], h = host_out_[3 * out_n_ + i];
      const float x0 = (cx - w / 2 - px) / s, y0 = (cy - h / 2 - py) / s;
      boxes.emplace_back(static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(w / s), static_cast<int>(h / s));
      scores.push_back(bs);
      classes.push_back(best);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, cfg_.score_threshold, cfg_.nms_threshold, keep);
    for (int k : keep) {
      Detection d;
      d.box = cv::Rect2f(boxes[k]) & cv::Rect2f(0, 0, static_cast<float>(bgr.cols), static_cast<float>(bgr.rows));
      d.score = scores[k];
      d.class_id = classes[k];
      out.push_back(d);
    }
    return out;
  }

  std::string name() const override {return "tensorrt";}

private:
  DetectorConfig cfg_;
  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx_;
  std::string in_name_, out_name_;
  int in_w_ = 640, in_h_ = 640, out_c_ = 0, out_n_ = 0;
  void * d_in_ = nullptr;
  void * d_out_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::vector<float> host_out_;
};

std::unique_ptr<Detector> make_tensorrt_detector(const DetectorConfig & c, std::string & info)
{
  auto d = std::make_unique<TensorRTDetector>();
  if (!d->init(c, info)) {return nullptr;}
  return d;
}

}  // namespace perception

#endif  // USE_TENSORRT
