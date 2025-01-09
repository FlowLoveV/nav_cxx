#pragma once

#include <cmath>
#include <concepts>
#include <numbers>

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp {

enum class NAVP_EXPORT AngleEnum : u8 {
  Degress,
  Radians,
  Dms,
  Nmea,
};

constexpr f64 CST_PI = std::numbers::pi;
constexpr f64 CST_D2R = CST_PI / 180.0;
constexpr f64 CST_R2D = 180.0 / CST_PI;

// turn a degress to a radians
template <std::floating_point T>
NAVP_EXPORT constexpr inline T to_degress(const T rad) noexcept {
  return rad * CST_R2D;
}

// turn a radians to a degress
template <std::floating_point T>
NAVP_EXPORT constexpr inline T to_radians(const T deg) noexcept {
  return deg * CST_D2R;
}

// Degress Minute Second
// # example
// -  30.5° <=> 30°30'0'
template <std::floating_point T>
struct NAVP_EXPORT DDmmss {
  bool negative;
  u8 mm;
  u16 hh;
  T ss;

  template <std::floating_point U = T>
    requires std::is_same_v<std::common_type_t<U, T>, T>
  static constexpr DDmmss from_degress(U deg) {
    bool negative = false;
    if (deg < 0) {
      negative = true;
      deg = -deg;
    }
    auto hh = static_cast<u16>(deg);
    T temp1 = (deg - static_cast<T>(hh)) * 60;
    auto mm = static_cast<u8>(temp1);
    T ss = (temp1 - static_cast<T>(mm)) * 60;
    return DDmmss{.negative = negative, .mm = mm, .hh = hh, .ss = ss};
  }

  template <std::floating_point U = T>
    requires std::is_same_v<std::common_type_t<U, T>, T>
  static constexpr DDmmss from_radians(U rad) {
    return DDmmss::from_degress<U>(navp::to_degress(rad));
  }

  template <std::floating_point U = T>
    requires std::is_same_v<std::common_type_t<U, T>, U>
  constexpr U to_degress() noexcept {
    return this->hh + this->mm / 60.0 + this->ss / 3600.0;
  }

  template <std::floating_point U = T>
    requires std::is_same_v<std::common_type_t<U, T>, U>
  constexpr U to_radians() noexcept {
    return navp::to_radians(this->to_degress());
  }
};

// nmea style hhmm.mm or hhhmm.mm -> dms
template <std::floating_point T>
NAVP_EXPORT DDmmss<T> nmea_style_dms_to_dms(const T hhmm) {
  T intPart, fracPart;
  fracPart = std::modf(hhmm, &intPart);
  T ss = fracPart * 60;
  T temp = intPart / 100.0;
  fracPart = std::modf(temp, &intPart);
  T hh = intPart;
  T mm = fracPart * 100;
  return DDmmss<T>{hh, mm, ss};
}

// dms -> nmea style hhmm.mm or hhhmm.mm
template <std::floating_point T>
NAVP_EXPORT T dms_to_nmea_style_dms(const DDmmss<T> &dms) {
  return (dms.hh * 100 + dms.mm + dms.ss / 60.0);
}

// nmea style hhmm.mm or hhhmm.mm -> deg
template <std::floating_point T>
NAVP_EXPORT T nmea_style_dms_to_degress(const T hhmm) {
  T intPart, fracPart;
  fracPart = std::modf(hhmm / 100.0, &intPart);
  return intPart + fracPart / 3.0 * 5.0;
}

// deg -> nmea style hhmm.mm or hhhmm.mm
template <std::floating_point T>
NAVP_EXPORT T degress_to_nmea_style_dms(const T deg) {
  T intPart, fracPart;
  fracPart = std::modf(deg, &intPart);
  return intPart * 100 + fracPart * 60;
}

// nmea style hhmm.mm or hhhmm.mm -> rad
template <std::floating_point T>
NAVP_EXPORT T nmea_style_dms_to_radians(const T hhmm) {
  return to_radians(nmea_style_dms_to_degress(hhmm));
}

// rad -> nmea style hhmm.mm or hhhmm.mm
template <std::floating_point T>
NAVP_EXPORT T radians_to_nmea_style_dms(const T rad) {
  return degress_to_nmea_style_dms(to_degress(rad));
}

template <std::floating_point T>
NAVP_EXPORT std::tuple<T, T> sin_cos(const T f) {
  T sinValue = std::sin(f);
  T cosValue = std::cos(f);
  return {sinValue, cosValue};
}

}  // namespace navp