#pragma once

#include <cmath>
#include <concepts>
#include <numbers>

#include "types.hpp"

namespace navp {

enum class AngleEnum : u8 {
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
constexpr inline T to_degress(const T rad) noexcept {
  return rad * CST_R2D;
}

// turn a radians to a degress
template <std::floating_point T>
constexpr inline T to_radians(const T deg) noexcept {
  return deg * CST_D2R;
}

// Degress Minute Second
// # example
// -  30.5° <=> 30°30'0'
template <std::floating_point T>
struct DDmmss {
  T hh, mm, ss;

  template <std::floating_point U = T>
    requires std::is_same_v<std::common_type_t<U, T>, T>
  static constexpr DDmmss form_degress(U deg) {
    T hh = i32(deg);
    T temp1 = (deg - hh) * 60;
    T mm = i32(temp1);
    T temp2 = (temp1 - mm) * 60;
    T ss = i32(temp2);
    return DDmmss{hh, mm, ss};
  }

  template <std::floating_point U = T>
    requires std::is_same_v<std::common_type_t<U, T>, T>
  static constexpr DDmmss from_radians(U rad) {
    return DDmmss::from_degress(navp::to_degress(rad));
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
DDmmss<T> nmeaStyleDmsToDms(const T hhmm) {
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
T dmsToNmeaStyleDms(const DDmmss<T> &dms) {
  return (dms.hh * 100 + dms.mm + dms.ss / 60.0);
}

// nmea style hhmm.mm or hhhmm.mm -> deg
template <std::floating_point T>
T nmeaStyleDmsToDegress(const T hhmm) {
  T intPart, fracPart;
  fracPart = std::modf(hhmm / 100.0, &intPart);
  return intPart + fracPart / 3.0 * 5.0;
}

// deg -> nmea style hhmm.mm or hhhmm.mm
template <std::floating_point T>
T degressToNmeaStyleDms(const T deg) {
  T intPart, fracPart;
  fracPart = std::modf(deg, &intPart);
  return intPart * 100 + fracPart * 60;
}

// nmea style hhmm.mm or hhhmm.mm -> rad
template <std::floating_point T>
T nmeaStyleDmsToRadians(const T hhmm) {
  return to_radians(nmeaStyleDmsToDegress(hhmm));
}

// rad -> nmea style hhmm.mm or hhhmm.mm
template <std::floating_point T>
T radiansToNmeaStyleDms(const T rad) {
  return degressToNmeaStyleDms(to_degress(rad));
}

template <std::floating_point T>
std::tuple<T, T> sin_cos(const T f) {
  T sinValue = std::sin(f);
  T cosValue = std::cos(f);
  return {sinValue, cosValue};
}

}  // namespace navp