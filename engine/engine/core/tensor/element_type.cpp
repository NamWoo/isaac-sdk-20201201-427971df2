/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "element_type.hpp"

namespace isaac {

size_t ElementTypeByteCount(ElementType element_type) {
  switch (element_type) {
    case ElementType::kUnknown: return 0;
    case ElementType::kUInt8:   return 1;
    case ElementType::kUInt16:  return 2;
    case ElementType::kUInt32:  return 4;
    case ElementType::kUInt64:  return 8;
    case ElementType::kInt8:    return 1;
    case ElementType::kInt16:   return 2;
    case ElementType::kInt32:   return 4;
    case ElementType::kInt64:   return 8;
    case ElementType::kFloat16: return 2;
    case ElementType::kFloat32: return 4;
    case ElementType::kFloat64: return 8;
  }
  return 0;
}

#define ELEMENT_TYPE_C_STR_CASE(ELEMENT_TYPE) \
  case ELEMENT_TYPE: return #ELEMENT_TYPE;

const char* ElementTypeCStr(ElementType element_type) {
  switch (element_type) {
    case ElementType::kUnknown: return "(unknown)";
    ELEMENT_TYPE_C_STR_CASE(ElementType::kUInt8);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kUInt16);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kUInt32);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kUInt64);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kInt8);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kInt16);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kInt32);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kInt64);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kFloat16);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kFloat32);
    ELEMENT_TYPE_C_STR_CASE(ElementType::kFloat64);
  }
  return "(unknown)";
}

#undef ELEMENT_TYPE_C_STR_CASE

}  // namespace isaac
