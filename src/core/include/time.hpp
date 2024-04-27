#include <cstdint>
#include <ratio>

#include "macro.hpp"

// export module time, only valid when ENMODULES is defined
NAV_EXPORT NAV_MODULE NAV_MODULE_NAME(time);
NAV_IMPORT STD_MODULE;
// if don't enable modules, include files
#ifndef ENMODULES
#pragma once
#include <chrono>
#include <expected>
#endif

namespace nav {

using namespace std::chrono_literals;

NAV_EXPORT enum class TimeScale : uint8_t {
  /// TAI is the representation of an Epoch internally
  TAI,
  /// Terrestrial Time (TT) (previously called Terrestrial Dynamical Time (TDT))
  TT,
  /// Ephemeris Time as defined by SPICE (slightly different from true TDB)
  ET,
  /// Dynamic Barycentric Time (TDB) (higher fidelity SPICE ephemeris time)
  TDB,
  /// Universal Coordinated Time
  UTC,
  /// GPST Time also applies to QZSS, IRNSS and GAL constellations
  GPST,
  /// Galileo Time scale
  GST,
  /// BeiDou Time scale
  BDT,
};

struct period {
  std::chrono::duration<uint64_t, std::ratio<1>> second{0s};
  std::chrono::duration<double_t, std::nano> nanos{0s};
};

NAV_NODISCARD period RefEpochAtTaiZero(const TimeScale scale) NAV_NOEXCEPT {
  switch (scale) {
    case nav::TimeScale::TAI:
      return period{};
    case nav::TimeScale::TT:
      return period{{32s}, {0.184s}};
    case TimeScale::ET:
      return period{};
    case TimeScale::TDB:
      return period{};
    case TimeScale::UTC:
      return period{};
    case TimeScale::GPST:
      return period{};
    case TimeScale::GST:
      return period{};
    case TimeScale::BDT:
      return period{};
      break;
  }
}

template <nav::TimeScale scale = TimeScale::TAI>
NAV_EXPORT class Epoch {
 private:
  TimeScale _scale = scale;  // time scale
  period _period;            // time since reference epoch

 public:
  NAV_NODISCARD auto time_scale() NAV_NOEXCEPT -> nav::TimeScale {
    return _scale;
  }
};

NAV_EXPORT class Gpst;
NAV_EXPORT class Bdst;
NAV_EXPORT class Utc;
NAV_EXPORT class Mjd;
NAV_EXPORT class Doy;

NAV_EXPORT class Gpst {
 public:
  static constexpr auto scale = TimeScale::GPST;
};

NAV_EXPORT class Bdst {
 public:
  static constexpr auto scale = TimeScale::BDT;
};

NAV_EXPORT class Utc {
 public:
  static constexpr auto scale = TimeScale::UTC;
};

NAV_EXPORT class Mjd {
 public:
  static constexpr auto scale = TimeScale::TT;
};

NAV_EXPORT class Doy {
 public:
  static constexpr auto scale = TimeScale::TT;
};

}  // namespace nav
