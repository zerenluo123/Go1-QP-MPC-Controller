#pragma once

#include <Eigen/Dense>
#include <torch/script.h>


class TorchEigen{
  public:
    TorchEigen() = default;

    void load(std::string libraryPath);
    void run(const Eigen::VectorXf& input, Eigen::VectorXf& output);

  private:
    torch::jit::script::Module model_;
    torch::Tensor eigenToTensor(const Eigen::VectorXf& e);
    Eigen::VectorXf tensorToEigen(const torch::Tensor& t);
};
