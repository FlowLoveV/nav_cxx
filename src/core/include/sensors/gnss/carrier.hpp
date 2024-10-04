#pragma once

#include <boost/algorithm/string.hpp>
#include <boost/bimap.hpp>
#include <format>

#include "utils/error.hpp"
#include "utils/result.hpp"
#include "utils/types.hpp"

namespace navp::sensors::gnss {
enum class CarrierEnum : u8;
}

namespace navp::sensors::gnss {
extern const boost::bimap<std::string, navp::sensors::gnss::CarrierEnum> CARRIER_TABLE;
}

namespace navp::sensors::gnss {

enum class CarrierEnum : u8 {
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

struct Carrier {
  // Carrier() : id(CarrierEnum::L1) {}
  // Carrier(CarrierEnum carrier) : id(carrier) {}
  // from_str
  static NavResult<Carrier> from_str(const char* str) {
    std::string s(str);
    boost::algorithm::to_upper(s);
    boost::algorithm::trim(s);
    /*
     * GPS, Galieo
     */
    auto it = navp::sensors::gnss::CARRIER_TABLE.left.find(s);
    if (it != navp::sensors::gnss::CARRIER_TABLE.left.end()) {
      return Ok(Carrier{it->second});
    }
    return Err(errors::NavError::Utils::Gnss::ParseCarrierStringError);
  }

  CarrierEnum id;
};

}  // namespace navp::sensors::gnss

template <>
struct std::formatter<navp::sensors::gnss::Carrier, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <class FormatContext>
  auto format(navp::sensors::gnss::Carrier carrier, FormatContext& ctx) const {
    try {
      const std::string& carrier_str = navp::sensors::gnss::CARRIER_TABLE.right.at(carrier.id);
      return std::format_to(ctx.out(), "{}", carrier_str);
    } catch (const std::out_of_range&) {
      return std::format_to(ctx.out(), "Unknown Carrier");
    }
  }
};