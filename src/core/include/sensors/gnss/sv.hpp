#pragma once

#include <boost/algorithm/string.hpp>
#include <string>
#include <string_view>

#include "enums.hpp"
#include "magic_enum.hpp"
#include "utils/error.hpp"
#include "utils/logger.hpp"
#include "utils/macro.hpp"
#include "utils/result.hpp"
#include "utils/types.hpp"

namespace navp::sensors::gnss {

inline auto constexpr NSYSGPS = 1;
inline auto constexpr NSATGPS = 32;  ///< potential number of GPS satellites, PRN goes from 1 to this number
inline auto constexpr NSATGLO = 27;  ///< potential number of GLONASS satellites, PRN goes from 1 to this number
inline auto constexpr NSATGAL = 36;  ///< potential number of Galileo satellites, PRN goes from 1 to this number
inline auto constexpr NSATQZS = 7;   ///< potential number of QZSS satellites, PRN goes from 1 to this number
inline auto constexpr NSATLEO = 78;  ///< potential number of LEO satellites, PRN goes from 1 to this number
inline auto constexpr NSATBDS = 62;  ///< potential number of Beidou satellites, PRN goes from 1 to this number
inline auto constexpr NSATSBS = 39;  ///< potential number of SBAS satellites, PRN goes from 1 to this number

struct NAVP_EXPORT Constellation {
  // constexpr Constellation() : id(ConstellationEnum::GPS) {}
  // constexpr Constellation(ConstellationEnum id) : id(id) {}
  static NavResult<Constellation> form_str(const char* str) {
    std::string s(str);
    boost::algorithm::trim(s);
    boost::algorithm::to_lower(s);
    ConstellationEnum result;
    if (s == "g" || s == "gps")
      result = ConstellationEnum::GPS;
    else if (s == "c" || s == "bds")
      result = ConstellationEnum::BDS;
    else if (s == "e" || s == "gal")
      result = ConstellationEnum::GAL;
    else if (s == "r" || s == "glo")
      result = ConstellationEnum::GLO;
    else if (s == "j" || s == "qzss")
      result = ConstellationEnum::QZS;
    else if (s == "i" || s == "irnss")
      result = ConstellationEnum::IRN;
    else if (s == "s" || s == "sbas")
      result = ConstellationEnum::SBS;
    else if (s == "m" || s == "mixed")
      result = ConstellationEnum::Mixed;
    else if (s == "ausnz")
      result = ConstellationEnum::AusNZ;
    else if (s == "egnos")
      result = ConstellationEnum::EGNOS;
    else if (s == "waas")
      result = ConstellationEnum::WAAS;
    else if (s == "kass")
      result = ConstellationEnum::KASS;
    else if (s == "gagan")
      result = ConstellationEnum::GAGAN;
    else if (s == "asbas")
      result = ConstellationEnum::ASBAS;
    else if (s == "nsas")
      result = ConstellationEnum::NSAS;
    else if (s == "asal")
      result = ConstellationEnum::ASAL;
    else if (s == "msas")
      result = ConstellationEnum::MSAS;
    else if (s == "span")
      result = ConstellationEnum::SPAN;
    else if (s == "gbas")
      result = ConstellationEnum::GBAS;
    else if (s == "sdcm")
      result = ConstellationEnum::SDCM;
    else if (s == "bdsbas")
      result = ConstellationEnum::BDSBAS;
    else if (s.find("gps") != std::string::npos)
      result = ConstellationEnum::GPS;
    else if (s.find("glonass") != std::string::npos)
      result = ConstellationEnum::GLO;
    else if (s.find("beidou") != std::string::npos)
      result = ConstellationEnum::BDS;
    else if (s.find("galileo") != std::string::npos)
      result = ConstellationEnum::GAL;
    else if (s.find("qzss") != std::string::npos)
      result = ConstellationEnum::QZS;
    else if (s.find("sbas") != std::string::npos || s.find("geo") != std::string::npos)
      result = ConstellationEnum::SBS;
    else if (s.find("irnss") != std::string::npos || s.find("navic") != std::string::npos)
      result = ConstellationEnum::IRN;
    else if (s.find("mix") != std::string::npos)
      result = ConstellationEnum::Mixed;
    else {
      return Err(errors::NavError::Utils::Gnss::ParseConstellationStringError);
    }

    return Ok(Constellation(result));
  }
  bool is_sbas() const {
    return id == ConstellationEnum::WAAS || id == ConstellationEnum::EGNOS || id == ConstellationEnum::MSAS ||
           id == ConstellationEnum::GAGAN || id == ConstellationEnum::BDSBAS || id == ConstellationEnum::KASS ||
           id == ConstellationEnum::SDCM || id == ConstellationEnum::ASBAS || id == ConstellationEnum::SPAN ||
           id == ConstellationEnum::SBS || id == ConstellationEnum::AusNZ || id == ConstellationEnum::GBAS ||
           id == ConstellationEnum::NSAS || id == ConstellationEnum::ASAL;
  }
  bool is_mixed() const { return id == ConstellationEnum::Mixed; }
  constexpr std::string_view name() const { return magic_enum::enum_name(id); }
  constexpr bool operator==(const Constellation& rhs) const { return this->id == rhs.id; }
  constexpr bool operator!=(const Constellation& rhs) const { return this->id != rhs.id; }
  constexpr auto operator<=>(const Constellation& rhs) const { return id <=> rhs.id; }

  ConstellationEnum id;
};

struct NAVP_EXPORT Sv {
  // Sv(u8 prn, ConstellationEnum cons_enum) : prn(prn), constellation(cons_enum) {}
  // Sv(u8 prn, Constellation cons) : prn(prn), constellation(cons) {}
  // Sv(ConstellationEnum cons_enum) : prn(0), constellation(cons_enum) {}
  // Sv(Constellation cons) : prn(0), constellation(cons) {}
  static NavResult<Sv> from_str(const char* str) {
    std::string_view view(&str[1]);
    char first[] = {str[0], '\0'};
    auto constellation = Constellation::form_str(first);
    if (constellation.is_err()) {
      auto error_msg = std::format("Unable to parse string \"{}\" to Constellation", &str[0]);
      nav_error(error_msg);
      throw std::runtime_error(error_msg);
    }
    u8 prn;
    if (std::all_of(view.begin(), view.end(), [](char c) { return std::isspace(c); })) {
      prn = 0;
    } else {
      auto [ptr, ec] = std::from_chars(view.data(), view.data() + view.size(), prn);
      if (ec != std::errc()) {
        // auto error_msg = std::format("Unable to parse string \"{}\" to Sv", std::string(str.data() + 1));
        // nav_error(error_msg);
        // throw std::runtime_error(error_msg);
        return Err(errors::NavError::Utils::Gnss::ParseSvStringError);
      }
    }
    return Ok(Sv{prn, constellation.unwrap_unchecked()});
  };
  constexpr inline bool operator!=(const Sv& rhs) const noexcept { return !(*this == rhs); }
  constexpr inline bool operator==(const Sv& rhs) const noexcept {
    // When prn is 0, Sv degenerates into a Constellation identifier, which is used to determine
    // whether it is the same Constellation.
    if (this->prn == 0 || rhs.prn == 0) {
      return this->constellation == rhs.constellation;
    }
    return prn == rhs.prn && constellation == rhs.constellation;
  }
  constexpr inline auto operator<=>(const Sv& rhs) const {
    if (rhs.constellation != constellation) {
      return rhs.constellation.id <=> constellation.id;
      // auto error_msg = "Cannot compare prn of Sv with different systems.";
      // nav_error(error_msg);
      // throw std::runtime_error(error_msg);
    }
    return prn <=> rhs.prn;
  }

  operator bool() const noexcept { return !(constellation.id == ConstellationEnum::NONE || prn == 0); }

  u8 prn;
  Constellation constellation;
};

std::vector<Sv> get_sv_sats(ConstellationEnum cons);

}  // namespace navp::sensors::gnss

namespace std {
template <>
struct NAVP_EXPORT formatter<navp::sensors::gnss::Constellation, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <class FmtContext>
  FmtContext::iterator format(navp::sensors::gnss::Constellation cons, FmtContext& ctx) const {
    using navp::sensors::gnss::ConstellationEnum;
    switch (cons.id) {
      case ConstellationEnum::GPS:
        return std::format_to(ctx.out(), "G");
      case ConstellationEnum::GLO:
        return std::format_to(ctx.out(), "R");
      case ConstellationEnum::GAL:
        return std::format_to(ctx.out(), "E");
      case ConstellationEnum::BDS:
        return std::format_to(ctx.out(), "C");
      case ConstellationEnum::QZS:
        return std::format_to(ctx.out(), "J");
      case ConstellationEnum::IRN:
        return std::format_to(ctx.out(), "I");
      default: {
        if (cons.is_sbas()) {
          return std::format_to(ctx.out(), "S");
        } else {
          return std::format_to(ctx.out(), "-");
        }
      }
    }
  }
};

template <>
struct NAVP_EXPORT formatter<navp::sensors::gnss::Sv, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }
  template <class FormatContext>
  auto format(navp::sensors::gnss::Sv sv, FormatContext& ctx) const {
    if (sv.prn == 0) {
      return std::format_to(ctx.out(), "{}--", sv.constellation);
    }
    return std::format_to(ctx.out(), "{}{:02d}", sv.constellation, sv.prn);
  }
};

template <>
struct NAVP_EXPORT hash<navp::sensors::gnss::Sv> {
  size_t operator()(const navp::sensors::gnss::Sv& sv) const {
    return hash<navp::u8>()(sv.prn) ^ (hash<navp::u8>()((navp::u8)(sv.constellation.id)) << 1);
  }
};
}  // namespace std
