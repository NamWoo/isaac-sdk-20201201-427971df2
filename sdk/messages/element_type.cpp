/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "element_type.hpp"

isaac::ElementType FromProto(::ElementType proto_element_type) {
  switch (proto_element_type) {
    case ::ElementType::UINT8:   return isaac::ElementType::kUInt8;
    case ::ElementType::UINT16:  return isaac::ElementType::kUInt16;
    case ::ElementType::UINT32:  return isaac::ElementType::kUInt32;
    case ::ElementType::UINT64:  return isaac::ElementType::kUInt64;
    case ::ElementType::INT8:    return isaac::ElementType::kInt8;
    case ::ElementType::INT16:   return isaac::ElementType::kInt16;
    case ::ElementType::INT32:   return isaac::ElementType::kInt32;
    case ::ElementType::INT64:   return isaac::ElementType::kInt64;
    case ::ElementType::FLOAT16: return isaac::ElementType::kFloat16;
    case ::ElementType::FLOAT32: return isaac::ElementType::kFloat32;
    case ::ElementType::FLOAT64: return isaac::ElementType::kFloat64;
    case ::ElementType::UNKNOWN: return isaac::ElementType::kUnknown;
  }
  return isaac::ElementType::kUnknown;
}

::ElementType ToProto(isaac::ElementType element_type) {
  switch (element_type) {
    case isaac::ElementType::kUInt8:   return ::ElementType::UINT8;
    case isaac::ElementType::kUInt16:  return ::ElementType::UINT16;
    case isaac::ElementType::kUInt32:  return ::ElementType::UINT32;
    case isaac::ElementType::kUInt64:  return ::ElementType::UINT64;
    case isaac::ElementType::kInt8:    return ::ElementType::INT8;
    case isaac::ElementType::kInt16:   return ::ElementType::INT16;
    case isaac::ElementType::kInt32:   return ::ElementType::INT32;
    case isaac::ElementType::kInt64:   return ::ElementType::INT64;
    case isaac::ElementType::kFloat16: return ::ElementType::FLOAT16;
    case isaac::ElementType::kFloat32: return ::ElementType::FLOAT32;
    case isaac::ElementType::kFloat64: return ::ElementType::FLOAT64;
    case isaac::ElementType::kUnknown: return ::ElementType::UNKNOWN;
  }
  return ::ElementType::UNKNOWN;
}
