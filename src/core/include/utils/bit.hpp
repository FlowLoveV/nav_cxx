#pragma once

#include <concepts>

#include "utils/types.hpp"

namespace navp::utils {

// unchecked extract bits
template <std::integral T>
[[__nodiscard__("unused function result")]] constexpr T extract_bits(T value, u8 index, u8 n) noexcept {
  T mask = ((T)1 << n) - 1;
  return (value >> index) & mask;
}

}  // namespace navp::utils