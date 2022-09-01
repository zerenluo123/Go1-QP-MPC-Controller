#pragma once

// @generated by torchgen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>



#include <ATen/ops/index_ops.h>

namespace at {


// aten::index.Tensor(Tensor self, Tensor?[] indices) -> Tensor
TORCH_API inline at::Tensor index(const at::Tensor & self, const c10::List<c10::optional<at::Tensor>> & indices) {
    return at::_ops::index_Tensor::call(self, indices);
}

}
