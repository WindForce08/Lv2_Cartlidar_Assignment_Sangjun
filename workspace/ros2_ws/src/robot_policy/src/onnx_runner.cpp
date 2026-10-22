// ONNX Runtime CPU 추론 (USE_ONNXRUNTIME 일 때만). 입력 1 × obs_dim, 출력 1 × action_dim (float32)
#include "robot_policy/policy.hpp"

#ifdef USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace robot_policy
{

#ifdef USE_ONNXRUNTIME
class OnnxRunner : public PolicyRunner
{
public:
  bool init(const std::string & path, std::string & err)
  {
    try {
      Ort::SessionOptions so;
      so.SetIntraOpNumThreads(1);            // 작은 MLP — 스레드 1개가 지연이 가장 짧고 안정적
      so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(env(), path.c_str(), so);
      Ort::AllocatorWithDefaultOptions alloc;
      in_name_ = session_->GetInputNameAllocated(0, alloc).get();
      out_name_ = session_->GetOutputNameAllocated(0, alloc).get();
      const auto in_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      const auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      in_dim_ = static_cast<int>(in_shape.back());
      out_dim_ = static_cast<int>(out_shape.back());
      return in_dim_ > 0 && out_dim_ > 0;
    } catch (const std::exception & e) {
      err = e.what();
      return false;
    }
  }

  bool run(const std::vector<float> & obs, std::vector<float> & action) override
  {
    try {
      const std::array<int64_t, 2> shape{1, static_cast<int64_t>(obs.size())};
      auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      Ort::Value in = Ort::Value::CreateTensor<float>(mem, const_cast<float *>(obs.data()), obs.size(), shape.data(), 2);
      const char * in_names[] = {in_name_.c_str()};
      const char * out_names[] = {out_name_.c_str()};
      auto out = session_->Run(Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
      const float * p = out[0].GetTensorData<float>();
      action.assign(p, p + out_dim_);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }
  int input_dim() const override {return in_dim_;}
  int output_dim() const override {return out_dim_;}

private:
  static Ort::Env & env()
  {
    static Ort::Env e(ORT_LOGGING_LEVEL_WARNING, "robot_policy");
    return e;
  }
  std::unique_ptr<Ort::Session> session_;
  std::string in_name_, out_name_;
  int in_dim_ = 0, out_dim_ = 0;
};
#endif

std::unique_ptr<PolicyRunner> make_onnx_runner(const std::string & model_path, std::string & err)
{
#ifdef USE_ONNXRUNTIME
  auto r = std::make_unique<OnnxRunner>();
  if (!r->init(model_path, err)) {return nullptr;}
  return r;
#else
  (void)model_path;
  err = "ONNX Runtime 없이 빌드됨";
  return nullptr;
#endif
}

bool PolicyLibrary::onnx_available()
{
#ifdef USE_ONNXRUNTIME
  return true;
#else
  return false;
#endif
}

}  // namespace robot_policy
