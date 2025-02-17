#pragma once

#include <format>
#include <magic_enum.hpp>

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/gnss_exception.hpp"
#include "utils/macro.hpp"
#include "utils/result.hpp"
#include "utils/types.hpp"

namespace navp::sensors::gnss {

struct NAVP_EXPORT Constellation {
  static Result<Constellation, GnssParseConstellationError> form_str(const char* str);

  bool is_sbas() const;

  bool is_mixed() const;

  std::string_view name() const;

  bool operator==(const Constellation& rhs) const;

  bool operator!=(const Constellation& rhs) const;

  std::strong_ordering operator<=>(const Constellation& rhs) const;

  ConstellationEnum id;
};

struct NAVP_EXPORT Sv {
  // runtime error may contain :
  // - GnssParseConstellationError
  // - GnssParseSvError
  static Result<Sv, GnssRuntimeError> from_str(const char* str);

  bool operator!=(const Sv& rhs) const noexcept;

  bool operator==(const Sv& rhs) const noexcept;

  std::strong_ordering operator<=>(const Sv& rhs) const;

  operator bool() const noexcept;

  ConstellationEnum system() const noexcept;

  ConstellationEnum& system() noexcept;

  u8 prn;
  Constellation constellation;
};

std::vector<Sv> get_sv_sats(ConstellationEnum cons);

}  // namespace navp::sensors::gnss

template <>
struct NAVP_EXPORT std::formatter<navp::sensors::gnss::Constellation, char> {
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
struct NAVP_EXPORT std::formatter<navp::sensors::gnss::Sv, char> {
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
struct NAVP_EXPORT std::hash<navp::sensors::gnss::Sv> {
  size_t operator()(const navp::sensors::gnss::Sv& sv) const {
    return hash<navp::u8>()(sv.prn) ^ (hash<navp::u8>()((navp::u8)(sv.system())) << 1);
  }
};
