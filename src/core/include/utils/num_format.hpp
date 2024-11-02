#pragma once

#include <string>
#include "utils/types.hpp"
#include "utils/macro.hpp"

namespace navp::utils {

enum class NAVP_EXPORT AlignmentDirectionEnum { Left, Right };

enum class NAVP_EXPORT NumTypeEnum {
  Integer,
  Float,
};

template <AlignmentDirectionEnum align, NumTypeEnum num_type, u8 width, u8 precision>
struct NAVP_EXPORT NumFormatter {
  template <typename T>
  constexpr std::string format(T val) const noexcept {
    if constexpr (align == AlignmentDirectionEnum::Left) {
      if constexpr (num_type == NumTypeEnum::Integer) {
        return std::format("{:<{}}", val, width);
      } else {
        return std::format("{:<{}.{}f}", val, width, precision);
      }
    } else {
      if constexpr (num_type == NumTypeEnum::Integer) {
        return std::format("{:>{}}", val, width);
      } else {
        return std::format("{:>{}.{}f}", val, width, precision);
      }
    }
  }
};

}  // namespace navp::utils