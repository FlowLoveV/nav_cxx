#pragma once
#include <cassert>
#include <charconv>
#include <string>

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::utils {

constexpr i64 NAVP_EXPORT attoseconds_from(i64 val, i64 exponent = 0) {
  i64 scale = 1;
  while (exponent > 0) {
    scale *= 10;
    exponent--;
  }
  return val * (1'000'000'000'000'000'000 / scale);
}

template <typename _Tp>
constexpr _Tp from_chars(const char* first, const char* last, _Tp& value) {
  auto [ptr, ec] = std::from_chars(first, last, value);
  return ec == std::errc{} ? ptr : first;
}

}  // namespace navp::utils