#pragma once

#include <boost/bimap.hpp>

namespace nav {

enum class CarrierEnum : uint8_t {
  /// L1 (GPS, SBAS, QZSS)
  L1,
  /// L2 (GPS, QZSS)
  L2,
  /// L5 (GPS, SBAS), QZSS
  L5,
  /// L6 (LEX) QZSS
  L6,
  /// Glonass channel 1 with possible offset
  G1,
  /// Glonass G1a
  G1a,
  /// Glonass channel 2 with possible offset
  G2,
  /// Glonass G2a
  G2a,
  /// Glonass channel 3
  G3,
  /// E1 (Galileo)
  E1,
  /// E5 (Galileo)
  E5,
  /// E5a (Galileo)
  E5a,
  /// E5b (Galileo)
  E5b,
  /// E6 (Galileo military)
  E6,
  /// B1 (BDS)
  B1I,
  /// B1A (BDS)
  B1A,
  /// B1C (BDS)
  B1C,
  /// B2 (BDS)
  B2,
  /// B2i: BeiDou 2i
  B2I,
  /// B2a: BeiDou 2A
  B2A,
  /// B2b: BeiDou 2b
  B2B,
  /// B3
  B3,
  /// B3A
  B3A,
  /// IRNSS S
  S,
  /// DORIS S1 Frequency
  S1,
  /// DORIS U2 Frequency
  U2,
};

namespace constants {

extern const boost::bimap<std::string, CarrierEnum> CARRIER_TABLE;

}

}  // namespace nav