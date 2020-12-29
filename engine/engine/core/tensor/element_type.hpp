/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <iterator>

namespace isaac {

// Possible element types for a tensor.
enum class ElementType {
  kUnknown,
  kUInt8,
  kUInt16,
  kUInt32,
  kUInt64,
  kInt8,
  kInt16,
  kInt32,
  kInt64,
  kFloat16,
  kFloat32,
  kFloat64
};

// Gets the element type for a template parameter
template <typename K>
ElementType GetElementType();

// Gets the number of bytes required to store an element of the given type
size_t ElementTypeByteCount(ElementType element_type);

// Gets a string representation for the element type
const char* ElementTypeCStr(ElementType element_type);

namespace element_type_details {

// Helper type to implement GetElementType
template <typename K>
struct GetElementTypeImpl;

#define GET_ELEMENT_TYPE_IMPL(CPP_TYPE, ELEMENT_TYPE) \
  template <> struct GetElementTypeImpl<CPP_TYPE> { static const ElementType type = ELEMENT_TYPE; };

GET_ELEMENT_TYPE_IMPL(uint8_t,  ElementType::kUInt8);
GET_ELEMENT_TYPE_IMPL(uint16_t, ElementType::kUInt16);
GET_ELEMENT_TYPE_IMPL(uint32_t, ElementType::kUInt32);
GET_ELEMENT_TYPE_IMPL(uint64_t, ElementType::kUInt64);
GET_ELEMENT_TYPE_IMPL(int8_t,   ElementType::kInt8);
GET_ELEMENT_TYPE_IMPL(int16_t,  ElementType::kInt16);
GET_ELEMENT_TYPE_IMPL(int32_t,  ElementType::kInt32);
GET_ELEMENT_TYPE_IMPL(int64_t,  ElementType::kInt64);
GET_ELEMENT_TYPE_IMPL(float,    ElementType::kFloat32);
GET_ELEMENT_TYPE_IMPL(double,   ElementType::kFloat64);
GET_ELEMENT_TYPE_IMPL(char,     ElementType::kInt8);

#undef GET_ELEMENT_TYPE_IMPL

}  // namespace element_type_details

template <typename K>
ElementType GetElementType() { return element_type_details::GetElementTypeImpl<K>::type; }

}  // namespace isaac
