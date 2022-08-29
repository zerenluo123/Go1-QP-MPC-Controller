#include "torch_eigen/TorchEigen.hpp"


void TorchEigen::load(std::string libraryPath) {
    // load in the library
    model_ = torch::jit::load(libraryPath);
    model_.eval();
}

void TorchEigen::run(const Eigen::VectorXf &input, Eigen::VectorXf &output) {
  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> T;
  T.push_back(eigenToTensor(input).reshape({1, input.rows()}));
  at::Tensor model_out = model_.forward(T).toTensor();
  output = tensorToEigen(model_out);
}


torch::Tensor TorchEigen::eigenToTensor(const Eigen::VectorXf& e) {
  auto t = torch::empty({1, e.rows()});
  for (int i=0; i<e.size(); i++)
    t.index({0, i}) = e[i];
  return t;
}

Eigen::VectorXf TorchEigen::tensorToEigen(const torch::Tensor& t) {
  int len = t.size(1);
  Eigen::VectorXf e(len);
  for (int i=0; i<len; i++)
    e[i] = t.index({0, i}).item<float>();
  return e;
}
