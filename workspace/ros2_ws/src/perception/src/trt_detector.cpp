// TensorRTDetector — YOLOv8 형식 출력 엔진 (YOLO11 도 같음. 입력 1×3×H×W float, 출력 1×(4+nc)×N: cx, cy, w, h, 클래스 점수)
// TensorRT 10 API. USE_TENSORRT 일 때만 빌드된다. 후처리는 yolo_postprocess.hpp
//   엔진 만들기 예(FP32, 엔진을 쓸 기기에서 만든다): trtexec --onnx=yolo11n_task4_B.onnx --noTF32 --saveEngine=yolo11n_task4_B.engine
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
#include "perception/yolo_postprocess.hpp"

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

    int n_in = 0, n_out = 0;
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * n = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(n) == nvinfer1::TensorIOMode::kINPUT) {in_name_ = n; ++n_in;} else {out_name_ = n; ++n_out;}
    }
    if (n_in != 1 || n_out != 1) {info = "입력 1개 · 출력 1개 엔진만 씀 (NMS 를 넣은 엔진이 아닌지 확인)"; return false;}
    const auto in_dims = engine_->getTensorShape(in_name_.c_str());
    const auto out_dims = engine_->getTensorShape(out_name_.c_str());
    if (in_dims.nbDims != 4 || out_dims.nbDims != 3) {info = "입출력 모양이 YOLOv8 형식이 아님"; return false;}
    // 정적 모양 · float32 만 (동적 모양이면 d 가 -1, FP16 입출력이면 복사 크기가 틀림)
    if (in_dims.d[0] != 1 || in_dims.d[1] != 3 || in_dims.d[2] <= 0 || in_dims.d[3] <= 0 ||
      out_dims.d[0] != 1 || out_dims.d[2] <= 0 ||
      engine_->getTensorDataType(in_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
      engine_->getTensorDataType(out_name_.c_str()) != nvinfer1::DataType::kFLOAT)
    {
      info = "입출력이 정적 float32 1×3×H×W / 1×(4+클래스 수)×N 이 아님";
      return false;
    }
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

    return decode_yolo(host_out_, out_c_, out_n_, s, px, py, bgr.cols, bgr.rows,
      cfg_.engine_score_threshold, cfg_.nms_threshold, cfg_.target_class);
  }

  std::string name() const override {return "tensorrt";}
  int num_classes() const override {return out_c_ - 4;}

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
