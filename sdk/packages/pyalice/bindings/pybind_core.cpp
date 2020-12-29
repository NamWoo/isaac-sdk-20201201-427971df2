/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_core.hpp"

#include <vector>

#include "engine/core/buffers/buffer.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

namespace {

// Gets Python buffer format string
const char* PythonBufferFormatString(ElementType element_type) {
  switch (element_type) {
    case ElementType::kUnknown: return "";
    case ElementType::kUInt8: return "B";
    case ElementType::kUInt16: return "H";
    case ElementType::kUInt32: return "I";
    case ElementType::kUInt64: return "Q";
    case ElementType::kInt8: return "b";
    case ElementType::kInt16: return "h";
    case ElementType::kInt32: return "i";
    case ElementType::kInt64: return "q";
    case ElementType::kFloat16: return "";
    case ElementType::kFloat32: return "f";
    case ElementType::kFloat64: return "d";
  }
  return "";
}

}  // namespace

void InitPybindCore(pybind11::module& m) {
  pybind11::class_<CpuBufferConstView>(m, "CpuBufferConstView", pybind11::buffer_protocol())
      .def_buffer([](CpuBufferConstView buffer) -> pybind11::buffer_info {
        // Treat buffers as pure byte buffers
        const ssize_t size = static_cast<ssize_t>(buffer.size());
        return pybind11::buffer_info(
          const_cast<void*>(static_cast<const void*>(buffer.begin())), 1,
          "B", 1, {size}, {1}
        );
      });

  pybind11::class_<CpuUniversalTensorConstView>(m, "CpuUniversalTensorConstView",
                                                pybind11::buffer_protocol())
      .def_buffer([](CpuUniversalTensorConstView tensor) -> pybind11::buffer_info {
        if (tensor.element_type() == ElementType::kUnknown) {
          return pybind11::buffer_info(nullptr, 1, "B", 1, {0}, {1});
        }

        // Size of a single element in bytes
        const ssize_t element_size = ElementTypeByteCount(tensor.element_type());

        // Python format string
        const char* format = PythonBufferFormatString(tensor.element_type());

        // Gets tensor dimensions as std::vector
        std::vector<ssize_t> shape(tensor.dimensions().size());
        for (size_t i = 0; i < shape.size(); i++) {
          shape[i] = tensor.dimensions()[i];
        }

        // Computes tensor strides in bytes
        std::vector<ssize_t> strides(tensor.dimensions().size());
        for (size_t i = 0; i < shape.size(); i++) {
          strides[i] = element_size;
          for (size_t j = i + 1; j < shape.size(); j++) {
            strides[i] *= shape[j];
          }
        }

        // Return buffer object with tensor info
        return pybind11::buffer_info(
          const_cast<void*>(static_cast<const void*>(tensor.buffer().begin())), element_size,
          format, shape.size(), shape, strides
        );
      });
}

}  // namespace alice
}  // namespace isaac
