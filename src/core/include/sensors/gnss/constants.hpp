#pragma once

#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

namespace details {

// clang-format off
#define SET_FREQUENCY(freq, val) [static_cast<u8>(FreTypeEnum::freq)] = val
static f64 FREQ_ARRAY[]{
      SET_FREQUENCY(FTYPE_NONE, 0),  SET_FREQUENCY(F1, 1575.42E6),   SET_FREQUENCY(F2, 1227.60E6),
      SET_FREQUENCY(F5, 1176.45E6),  SET_FREQUENCY(F6, 1278.75E6),   SET_FREQUENCY(F7, 1207.14E6),
      SET_FREQUENCY(F8, 1191.795E6), SET_FREQUENCY(G1, 1602E6),      SET_FREQUENCY(G2, 1246E6),
      SET_FREQUENCY(G3, 1202.025E6), SET_FREQUENCY(G1A, 1600.995E6), SET_FREQUENCY(G2A, 1248.06E6),
      SET_FREQUENCY(B1, 1561.098E6), SET_FREQUENCY(B3, 1268.52E6),   SET_FREQUENCY(I9, 2492.028E6)};
#undef SET_FREQUENCY
struct Code2Freq {
// ref https://files.igs.org/pub/data/format/rinex_4.02.pdf
#define CASE_CODE(code) case ObsCodeEnum::code:   
  static constexpr FreTypeEnum gps_code_freq(ObsCodeEnum code) {
    switch (code) {
      CASE_CODE(L1C) CASE_CODE(L1P) CASE_CODE(L1W) CASE_CODE(L1Y)
      CASE_CODE(L1M) CASE_CODE(L1N) CASE_CODE(L1S) CASE_CODE(L1L)
      CASE_CODE(L1X) CASE_CODE(L1R)
          return FreTypeEnum::L1;
      CASE_CODE(L2C) CASE_CODE(L2D) CASE_CODE(L2S) CASE_CODE(L2L)
      CASE_CODE(L2X) CASE_CODE(L2P) CASE_CODE(L2W) CASE_CODE(L2Y)
      CASE_CODE(L2M) CASE_CODE(L2N) CASE_CODE(L2R)
          return FreTypeEnum::L2;
      CASE_CODE(L5I) CASE_CODE(L5Q) CASE_CODE(L5X)
          return FreTypeEnum::L5;
      default:
          return FreTypeEnum::FTYPE_NONE;
    }
  }
  static constexpr FreTypeEnum bds_code_freq(ObsCodeEnum code) {
    switch (code) {
      CASE_CODE(L2I) CASE_CODE(L2Q) CASE_CODE(L2X)  // BDS-2/3 Signals B1
          return FreTypeEnum::B1;
      CASE_CODE(L1D) CASE_CODE(L1P) CASE_CODE(L1X)  // BDS-3 Signals B1C
      CASE_CODE(L1S) CASE_CODE(L1L) CASE_CODE(L1Z)  // BDS-3 Signals B1A
          return FreTypeEnum::F1;
      CASE_CODE(L5D) CASE_CODE(L5P) CASE_CODE(L5X)  // BDS-3 Signals B2a
          return FreTypeEnum::F5;
      CASE_CODE(L7I) CASE_CODE(L7Q) CASE_CODE(L7X)  // BDS-2 Signals B2
      CASE_CODE(L7D) CASE_CODE(L7P) CASE_CODE(L7Z)  // BDS-3 Signals B2b
          return FreTypeEnum::F7;
      CASE_CODE(L8D) CASE_CODE(L8P) CASE_CODE(L8X)  // BDS-3 Signals B2(B2a+B2b)
          return FreTypeEnum::F8;
      CASE_CODE(L6I) CASE_CODE(L6Q) CASE_CODE(L6X)  // BDS-2/3 Signals B3
      CASE_CODE(L6D) CASE_CODE(L6P) CASE_CODE(L6Z)  // BDS-3 Signals B3A
          return FreTypeEnum::B3;
      default:
          return FreTypeEnum::FTYPE_NONE;
    }
  }
static constexpr FreTypeEnum glo_code_freq(ObsCodeEnum code) {
  switch (code) {
    CASE_CODE(L1C) CASE_CODE(L1P)
        return FreTypeEnum::G1;
    CASE_CODE(L2C) CASE_CODE(L2P)
        return FreTypeEnum::G2;
    CASE_CODE(L3I) CASE_CODE(L3Q) CASE_CODE(L3X)
        return FreTypeEnum::G3;
    CASE_CODE(L4A) CASE_CODE(L4B) CASE_CODE(L4X)
        return FreTypeEnum::G1A;
    CASE_CODE(L6A) CASE_CODE(L6B) CASE_CODE(L6X)
        return FreTypeEnum::G2A;
    default:
        return FreTypeEnum::FTYPE_NONE;
  }
}
static constexpr FreTypeEnum gal_code_freq(ObsCodeEnum code) {
  switch (code) {
    CASE_CODE(L1C) CASE_CODE(L1A) CASE_CODE(L1B)
    CASE_CODE(L1X) CASE_CODE(L1Z)
        return FreTypeEnum::E1;
    CASE_CODE(L5I) CASE_CODE(L5Q) CASE_CODE(L5X)
        return FreTypeEnum::E5A;
    CASE_CODE(L6A) CASE_CODE(L6B) CASE_CODE(L6C)
    CASE_CODE(L6X) CASE_CODE(L6Z) 
        return FreTypeEnum::E6;
    CASE_CODE(L7I) CASE_CODE(L7Q) CASE_CODE(L7X)
        return FreTypeEnum::E5B;
    CASE_CODE(L8I) CASE_CODE(L8Q) CASE_CODE(L8X)
        return FreTypeEnum::E5;
    default:
        return FreTypeEnum::FTYPE_NONE;
  }
}
static constexpr FreTypeEnum qzs_code_freq(ObsCodeEnum code) {
  switch (code) {
    CASE_CODE(L1C) CASE_CODE(L1S) CASE_CODE(L1L) CASE_CODE(L1E)
    CASE_CODE(L1B) CASE_CODE(L1X) CASE_CODE(L1Z)
        return FreTypeEnum::L1;
    CASE_CODE(L2S) CASE_CODE(L2L) CASE_CODE(L2X)
        return FreTypeEnum::L2;
    CASE_CODE(L5I) CASE_CODE(L5Q) CASE_CODE(L5X) CASE_CODE(L5D)
    CASE_CODE(L5P) CASE_CODE(L5Z)
        return FreTypeEnum::L5;
    CASE_CODE(L6X) CASE_CODE(L6Z) CASE_CODE(L6S) CASE_CODE(L6L)
    CASE_CODE(L6E)
        return FreTypeEnum::L6;
    default:
        return FreTypeEnum::FTYPE_NONE;
  }
}
static constexpr FreTypeEnum irn_code_freq(ObsCodeEnum code) {
  switch (code) {
    CASE_CODE(L1P) CASE_CODE(L1X) CASE_CODE(L1D)
        return FreTypeEnum::L1;
    CASE_CODE(L5X) CASE_CODE(L5A) CASE_CODE(L5B) CASE_CODE(L5C)
        return FreTypeEnum::L5;
    CASE_CODE(L9A) CASE_CODE(L9B) CASE_CODE(L9C) CASE_CODE(L9X)
        return FreTypeEnum::I9;
    default: 
        return FreTypeEnum::FTYPE_NONE;
  }
}
static constexpr FreTypeEnum sbs_code_freq(ObsCodeEnum code) {
  switch (code) {
    CASE_CODE(L1C)
        return FreTypeEnum::L1;
    CASE_CODE(L5I) CASE_CODE(L5Q) CASE_CODE(L5X)
        return FreTypeEnum::L5;
    default:
        return FreTypeEnum::FTYPE_NONE;
  }
}
#undef CASE_CODE 
};

// struct Freq2Carrier {
//   static CarrierEnum gps_freq_to_carrier(FreTypeEnum freq) {
//     switch (freq) {
//           case FreTypeEnum::L1:   return CarrierEnum::L1;
//           case FreTypeEnum::L2:   return CarrierEnum::L2;
//           case FreTypeEnum::L5:   return CarrierEnum::L5;
//           default:                return CarrierEnum::None;
//     }
//   };

//   static CarrierEnum gal_freq_to_carrier(FreTypeEnum freq) {
//     switch (freq) {
//           case FreTypeEnum::E1:   return CarrierEnum::E1;
//           case FreTypeEnum::E5A:  return CarrierEnum::E5a;
//           case FreTypeEnum::E5B:  return CarrierEnum::E5b;
//           case FreTypeEnum::E5:   return CarrierEnum::E5;
//           case FreTypeEnum::E6:   return CarrierEnum::E6;
//           default:                return CarrierEnum::None;
//     }
//   };

//   static CarrierEnum glo_freq_to_carrier(FreTypeEnum freq) {
//     switch (freq) {
//           case FreTypeEnum::G1:   return CarrierEnum::G1;
//           case FreTypeEnum::G2:   return CarrierEnum::G2;
//           case FreTypeEnum::G3:   return CarrierEnum::G3;
//           default:                return CarrierEnum::None;
//     }
//   }

//   static CarrierEnum bds_freq_to_carrier(FreTypeEnum freq) {
//     switch (freq) {

//           case FreTypeEnum::B1C:   return CarrierEnum::B1C;
//           case FreTypeEnum::B1A:   return CarrierEnum::B1A;
//           case FreTypeEnum::B2A:   return CarrierEnum::B2A;
//           case FreTypeEnum::B2:    return CarrierEnum::B2;
//           case FreTypeEnum::B2B:   return CarrierEnum::B2B;
//           case FreTypeEnum::B2AB:  return CarrierEnum::B2AB;
//           case FreTypeEnum::B3:    return CarrierEnum::B3;
//           default:                 return CarrierEnum::None;
//     }
//   }

//   static CarrierEnum qzs_freq_to_carrier(FreTypeEnum freq) {
//     switch (freq) {
//           case FreTypeEnum::L1:   return CarrierEnum::L1;
//           case FreTypeEnum::L2:   return CarrierEnum::L2;
//           case FreTypeEnum::L5:   return CarrierEnum::L5;
//           case FreTypeEnum::L6:   return CarrierEnum::L6;
//           default:                return CarrierEnum::None;
//     }
//   }

//   static CarrierEnum irn_freq_to_carrier(FreTypeEnum freq) {
//     switch (freq) {
//           case FreTypeEnum::L1:   return CarrierEnum::S;
//           case FreTypeEnum::L5:   return CarrierEnum::S;
//           case FreTypeEnum::I9:   return CarrierEnum::S;
//           default:                return CarrierEnum::None;
//     }
//   }
// };
// clang-format on

}  // namespace details

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

  static f64 gm(Sv sv) {
    switch (sv.system()) {
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

  static f64 omega(Sv sv) {
    switch (sv.system()) {
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
    switch (sv.system()) {
      case ConstellationEnum::BDS:
        return DtrF::BDS;
      case ConstellationEnum::GAL:
        return DtrF::GAL;
      default:
        return DtrF::GPS;
    }
  }

  static constexpr f64 max_toe(const Sv& sv) {
    switch (sv.system()) {
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

  inline static constexpr f64 frequency(FreTypeEnum freq_enum) noexcept {
    return details::FREQ_ARRAY[static_cast<u8>(freq_enum)];
  }

  inline static constexpr f64 wave_length(FreTypeEnum freq_enum) noexcept {
    return sensors::gnss::Constants::CLIGHT / frequency(freq_enum);
  }

  // clang-format off
  static constexpr FreTypeEnum code_to_freq_enum(Constellation sys, ObsCodeEnum code) noexcept {
    switch (sys.id) {
          case ConstellationEnum::GPS: return (details::Code2Freq::gps_code_freq(code));
          case ConstellationEnum::BDS: return (details::Code2Freq::bds_code_freq(code));
          case ConstellationEnum::GAL: return (details::Code2Freq::gal_code_freq(code));
          case ConstellationEnum::QZS: return (details::Code2Freq::qzs_code_freq(code));
          case ConstellationEnum::IRN: return (details::Code2Freq::irn_code_freq(code));
          case ConstellationEnum::SBS: return (details::Code2Freq::sbs_code_freq(code));
          default: return FreTypeEnum::FTYPE_NONE;
    }
  }
  // clang-format on

  inline static constexpr f64 code_to_freq(Constellation sys, ObsCodeEnum code) noexcept {
    return frequency(code_to_freq_enum(sys, code));
  }

  inline static constexpr f64 code_to_wave_length(Constellation sys, ObsCodeEnum code) noexcept {
    return wave_length(code_to_freq_enum(sys, code));
  }
};

}  // namespace navp::sensors::gnss
