#pragma once

#include "enums.hpp"
#include "sv.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

// GM(MU)
struct NAVP_EXPORT GM {
  inline static constexpr f64 GPS = 3.9860050E14;
  inline static constexpr f64 BDS = 3.986004418E14;
  inline static constexpr f64 GLO = 3.9860044E14;
  inline static constexpr f64 GAL = 3.986004418E14;
};

// Omega(Earth rotation speed)
struct NAVP_EXPORT Omega {
  inline static constexpr f64 GPS = 7.2921151467E-5;
  inline static constexpr f64 BDS = 7.292115E-5;
  inline static constexpr f64 GLO = 7.292115E-5;
  inline static constexpr f64 GAL = 7.2921151467E-5;
};

struct NAVP_EXPORT DtrF {
  inline static constexpr f64 GPS = -0.000000000444280763339306;
  inline static constexpr f64 BDS = -0.00000000044428073090439775;
  inline static constexpr f64 GAL = -0.00000000044428073090439775;
};

// Max Toe
struct NAVP_EXPORT MaxToe {
  inline static constexpr f64 GPS = 7200.0;
  inline static constexpr f64 QZS = 3600.0;
  inline static constexpr f64 GAL = 9600.0;
  inline static constexpr f64 BDS = 3600.0;
  inline static constexpr f64 GLO = 7200.0;
  inline static constexpr f64 SBAS = 360.0;
};

struct NAVP_EXPORT MaxIterNumber {
  inline static constexpr uint8_t KEPLER = 30;
};

struct NAVP_EXPORT Constants {
  inline static constexpr f64 CLIGHT = 299792458.0;

  static constexpr f64 gm(const Sv& sv) {
    switch (sv.constellation.id) {
      case ConstellationEnum::BDS:
        return GM::BDS;
      case ConstellationEnum::GAL:
        return GM::GAL;
      case ConstellationEnum::GLO:
        return GM::GLO;
      default:
        return GM::GPS;
    }
  }

  static constexpr f64 omega(const Sv& sv) {
    switch (sv.constellation.id) {
      case ConstellationEnum::BDS:
        return Omega::BDS;
      case ConstellationEnum::GAL:
        return Omega::GAL;
      case ConstellationEnum::GLO:
        return Omega::GLO;
      default:
        return Omega::GPS;
    }
  }

  static constexpr f64 dtr_f(const Sv& sv) {
    switch (sv.constellation.id) {
      case ConstellationEnum::BDS:
        return DtrF::BDS;
      case ConstellationEnum::GAL:
        return DtrF::GAL;
      default:
        return DtrF::GPS;
    }
  }

  static constexpr f64 max_toe(const Sv& sv) {
    switch (sv.constellation.id) {
      case ConstellationEnum::GPS:
        return MaxToe::GPS;
      case ConstellationEnum::GAL:
        return MaxToe::GAL;
      case ConstellationEnum::GLO:
        return MaxToe::GLO;
      case ConstellationEnum::QZS:
        return MaxToe::QZS;
      case ConstellationEnum::SBS:
        return MaxToe::SBAS;
      case ConstellationEnum::BDS:
        return MaxToe::BDS;
      default:
        return 0.0;
    }
  }
};

}  // namespace navp::sensors::gnss
