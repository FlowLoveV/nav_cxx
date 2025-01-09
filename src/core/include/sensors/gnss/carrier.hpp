#pragma once

#include <boost/algorithm/string.hpp>
#include <boost/bimap.hpp>
#include <format>

#include "magic_enum.hpp"
#include "sensors/gnss/gnss_exception.hpp"
#include "utils/macro.hpp"
#include "utils/result.hpp"
#include "utils/types.hpp"

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
  /// B2(B2a + B2b): Beidou 3
  B2AB,
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

struct NAVP_EXPORT Carrier {
  static Result<Carrier, GnssParseCarrierError> from_str(const char* str) {
    std::string s(str);
    boost::algorithm::to_upper(s);
    boost::algorithm::trim(s);
    /*
     * GPS, Galieo
     */
    auto result = magic_enum::enum_cast<CarrierEnum>(str);
    if (result.has_value()) {
      return result.value();
    }
    return Err(GnssParseCarrierError(std::format("can't parse unknown Carrier \'{}\'", str)));
  }

  constexpr auto operator<=>(const Carrier&) const = default;

  CarrierEnum id;
};

}  // namespace navp::sensors::gnss

template <>
struct NAVP_EXPORT std::formatter<navp::sensors::gnss::Carrier, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <class FormatContext>
  auto format(navp::sensors::gnss::Carrier carrier, FormatContext& ctx) const {
    return std::format_to(ctx.out(), "{}", magic_enum::enum_name(carrier.id));
  }
};