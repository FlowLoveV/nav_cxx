#pragma once

#include <algorithm>
#include <array>

#include "sensors/gnss/enums.hpp"
#include "utils/bit.hpp"
#include "utils/exception.hpp"
#include "utils/macro.hpp"

namespace navp {

REGISTER_NAV_RUNTIME_ERROR_CHILD(FgoRuntimeError, NavRuntimeError);
REGISTER_NAV_RUNTIME_ERROR_CHILD(FgoParameterBolckError, FgoRuntimeError);

using sensors::gnss::ConstellationEnum;

namespace details {
struct ParameterBlcokInfo {
  u8 index, len;
};
}  // namespace details

class ParameterBlcok;

class NAVP_EXPORT ParameterBlcok {
 public:
  constexpr bool has_extra_block() const noexcept { return block_record_ & (block_record_ + 1); }
  constexpr u8 count() const noexcept { return std::popcount(block_record_); }

 protected:
  u8 block_record_;
#define REGISTER_NAV_PARAMETER_BLOCK(name)                                                            \
 public:                                                                                              \
  inline constexpr void set_##name##_block(u8 begin, u8 len) noexcept {                               \
    check_valid(begin, len);                                                                          \
    check_owned(begin, len);                                                                          \
    name##_.index = begin;                                                                            \
    name##_.len = len;                                                                                \
    mark_block(begin, len);                                                                           \
  }                                                                                                   \
  inline constexpr u8 get_##name##_index(u8 offset) const noexcept { return name##_.index + offset; } \
                                                                                                      \
 protected:                                                                                           \
  details::ParameterBlcokInfo name##_;

 protected:
  constexpr void check_valid(u8 begin, u8 len) const {
    if (len == 0) {
      throw FgoParameterBolckError("Parameter block length should be greater than 0!");
    }
    if (begin + len > 255) {
      throw FgoParameterBolckError("Total parameter block length should be less than 255!");
    }
  }

  constexpr void check_owned(u8 begin, u8 len) const {
    auto extracted = utils::extract_bits(block_record_, begin, len);
    // extracted should be 0
    if (extracted | 0) {
      throw FgoParameterBolckError("Repeatedly occupying an already occupied parameter block!");
    }
  }

  constexpr void mark_block(u8 begin, u8 len) noexcept {
    for (u8 i = 0; i < len; ++i) block_record_ |= (1 << (begin + i));
  }

  constexpr void clear_block(u8 begin, u8 len) noexcept { block_record_ = 0; }

  /*
   * register parameter blocks here
   */
  REGISTER_NAV_PARAMETER_BLOCK(pos);       //> position
  REGISTER_NAV_PARAMETER_BLOCK(vel);       //> velocity
  REGISTER_NAV_PARAMETER_BLOCK(acc);       //> acceleration
  REGISTER_NAV_PARAMETER_BLOCK(att);       //> attitude
  REGISTER_NAV_PARAMETER_BLOCK(sv_clock);  //> satellite clock
  REGISTER_NAV_PARAMETER_BLOCK(rc_clcok);  //> receiver clcok

  REGISTER_NAV_PARAMETER_BLOCK(ar);    //> original observation ambiguity
  REGISTER_NAV_PARAMETER_BLOCK(sdar);  //> single difference ambiguity
  REGISTER_NAV_PARAMETER_BLOCK(ddar);  //> double difference ambiguity

#undef REGISTER_NAV_PARAMETER_BLOCK
};

struct ClockBlock {
  constexpr ClockBlock& enable(std::array<ConstellationEnum, 21>& sys) noexcept {
    std::fill(index_, index_ + 21, -1);
    std::sort(sys.begin(), sys.end());
    auto unique_end = std::unique(sys.begin(), sys.end());
    u8 len = 0;
    for (auto it = sys.begin(); it != unique_end; ++it) {
      index_[static_cast<u8>(*it)] = len++;
    }
    return *this;
  }

  constexpr u8 index(ConstellationEnum sys) noexcept { return index_[static_cast<u8>(sys)]; }

  constexpr u8 count() const noexcept { return 21 - std::count(std::begin(index_), std::end(index_), -1); }

 protected:
  i8 index_[21];
};

}  // namespace navp