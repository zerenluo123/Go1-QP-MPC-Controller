//
// Created by zerenluo on 29.08.22.
//

#include <torch/script.h>

#include <iostream>
#include <vector>
#include <memory>

/**
 * README:
 * In order to debug your Python to C++ conversion (i.e. PyTorch to Torchscript), you need to first have the following:
 *  - A pre-trained PyTorch neural network model (see module.py for an example of how to load a *.pth file in Python)
 *  - A build folder in python_debug/
 *  - An installation of LibTorch (C++11, No CUDA/CPU) in the directory pytorch_debug/libtorch from https://pytorch.org/
 *
 * In order to generate the torchscript file, you need to do the following:
 *  1) From the pytorch_debug/ folder, run 'python3 module.py' (no apostrophes). This code will generate the Torchscript file.
 *  2) Navigate to your pytorch_debug/build/ folder and run:
 *      a) cmake -DCMAKE_PREFIX_PATH=/path/to/libtorch ..
 *      b) make
 *  to build the libtorch_debug_app executable
 *  3) Then, run './libtorch_debug_app' from within the build/ folder in order to inference the network that has been serialized for Torchscript
 *
 * Note that this code and that in module.py is given only as an example of how to do the Python->C++ conversion of your network. You will need to adjust the code parameters and dimensions, etc. to suit your application.
 */

int main(int argc, const char* argv[]) {

    int stateDimension = 1221;

    torch::manual_seed(0);

    torch::jit::script::Module studentModule;

    studentModule = torch::jit::load("../student_traced_debug.pt");
    studentModule.to(torch::kCPU);
    studentModule.eval();
    torch::NoGradGuard no_grad_;

    std::vector<torch::jit::IValue> T;
    T.push_back(torch::ones({stateDimension}));

    auto actionTensor = studentModule.forward(T).toTensor();

    std::cout << actionTensor << std::endl;

    return 0;

}