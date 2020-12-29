/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <tuple>
#include <type_traits>

namespace isaac {

// TypelistContainsType derives from true_type if List contains Target, otherwise from false_type.
template <typename Target, typename... List>
struct TypelistContainsType;

template <typename Target, typename... List>
constexpr bool TypelistContains = TypelistContainsType<Target, List...>::value;

template <typename Target, typename Head, typename... Tail>
struct TypelistContainsType<Target, Head, Tail...> :
    std::conditional_t<std::is_same<Target, Head>::value,
                       std::true_type,
                       TypelistContainsType<Target, Tail...>
    > {};

template <typename Target>
struct TypelistContainsType<Target> : std::false_type {};

namespace details {
// UniqueImpl strips duplicate types from a typelist
template <typename...>
struct UniqueImpl;

template <typename Output>
struct UniqueImpl<std::tuple<>, Output> {
  using type = Output;
};

template <template <typename...> typename Pack,
    typename InputHead,
    typename... InputTail,
    typename... Output>
struct UniqueImpl<std::tuple<InputHead, InputTail...>, Pack<Output...>> :
    UniqueImpl<std::tuple<InputTail...>,
               typename std::conditional_t<TypelistContains<InputHead, Output...>,
                                           Pack<Output...>,
                                           Pack<Output..., InputHead>
               >> {};
}  // namespace details

// TypelistUnique's type is Pack<Input...> with any duplicate types in Input... removed.
template <template <typename...> typename Pack, typename... Input>
using TypelistUnique = typename details::UniqueImpl<std::tuple<Input...>, Pack<>>::type;

}  // namespace isaac
