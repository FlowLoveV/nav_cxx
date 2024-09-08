#pragma once

#include <boost/algorithm/string.hpp>
#include <expected>
#include <format>
#include <magic_enum.hpp>
#include <stdexcept>
#include <stdfloat>
#include <string_view>

#include "constants/carrier_table.hpp"
#include "errors.hpp"
#include "logger.hpp"

namespace navp {

template <typename T>
using Ref = const T&;
template <typename T>
using MutRef = T&;
template <typename T>
using ForwardRef = T&&;
template <typename T>
using constForwardRef = const T&&;

template <typename T>
using RefWrapper = std::reference_wrapper<T>;
template <typename T>
inline RefWrapper<T> RefWarp(Ref<T> t) {
  return std::ref(t);
}

template <typename T>
using Uptr = std::unique_ptr<T>;
template <typename T>
using Sptr = std::shared_ptr<T>;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

#ifdef __STDCPP_FLOAT16_T__
typedef std::float16_t f16;
#endif
typedef std::float_t f32;
typedef std::double_t f64;
#ifdef __STDCPP_FLOAT128_T__
typedef std::float128_t f128;
#endif
#ifdef __STDCPP_BFLOAT16_T__
typedef std::bfloat16_t bf16;
#endif

template <typename Ok>
using Result = std::expected<Ok, NavError>;

enum class BiographyEnum;
enum class LliFlagEnum : u8;
enum class EpochFlagEnum : u8;
enum class EphemerisTypeFlag : u8;
enum class ConstellationEnum : u8;
struct Carrier;
struct Constellation;
struct Sv;

// biography
enum class BiographyEnum {
  // https://files.igs.org/pub/data/format/rinex_4.00.pdf
  RINEX4,
};

enum class LliFlagEnum : u8 {
  // Current epoch is marked Ok or Unknown status
  OK_OR_UNKNOWN = 0x00,
  // Lock lost between previous observation and current observation,
  LOCK_LOSS = 0x01,
  // Half cycle slip marker
  HALF_CYCLE_SLIP = 0x02,
  // Observing under anti spoofing,
  // might suffer from decreased SNR - decreased signal quality
  UNDER_ANTI_SPOOFING = 0x03,
};

enum class EpochFlagEnum : u8 {
  /// Epoch is sane
  Ok = 0x00,
  /// Cycle slip at this epoch
  CycleSlip,
  /// Single too weak at this epoch
  WeakSingal,
  /// Incomplete observation at this epoch
  IncompleteObserve,
};

enum class EphemerisTypeFlag : u8 {
  /// Legacy navp message
  LNAV,
  /// Glonass FDMA message
  FDMA,
  /// Galileo FNAV message
  FNAV,
  /// Galileo INAV message
  INAV,
  /// IFNV,
  IFNV,
  /// BeiDou D1 navp message
  D1,
  /// BeiDou D2 navp message
  D2,
  /// D1D2
  D1D2,
  /// SBAS navp message
  SBAS,
  /// GPS / QZSS Civilian navp message
  CNAV,
  /// BeiDou CNV1 message
  CNV1,
  /// GPS / QZSS / BeiDou CNV2 message
  CNV2,
  /// BeiDou CNV3 message
  CNV3,
  /// CNVX special marker
  CNVX,
};

struct Carrier {
  Carrier() : id(CarrierEnum::L1) {}
  Carrier(CarrierEnum carrier) : id(carrier) {}
  // from_str
  static Carrier from_str(std::string_view str) {
    std::string s(str);
    boost::algorithm::to_upper(s);
    boost::algorithm::trim(s);
    /*
     * GPS, Galieo
     */
    auto it = navp::constants::CARRIER_TABLE.left.find(s);
    if (it != navp::constants::CARRIER_TABLE.left.end()) {
      return it->second;
    } else {
      auto error_msg = std::format("Unable to parse string \"{}\" to Carrier", str);
      nav_error(error_msg);
      throw std::runtime_error(error_msg);
    }
  }
  CarrierEnum id;
};

enum class ConstellationEnum : u8 {
  /// `GPS` american constellation,
  GPS = 0x00,
  /// `Glonass` russian constellation
  Glonass,
  /// `BeiDou` chinese constellation
  BeiDou,
  /// `QZSS` japanese constellation
  QZSS,
  /// `Galileo` european constellation
  Galileo,
  /// `IRNSS` constellation, renamed "NavIC"
  IRNSS,
  /// American augmentation system,
  WAAS,
  /// European augmentation system
  EGNOS,
  /// Japanese MTSAT Space Based augmentation system
  MSAS,
  /// Indian augmentation system
  GAGAN,
  /// Chinese augmentation system
  BDSBAS,
  /// South Korean augmentation system
  KASS,
  /// Russian augmentation system
  SDCM,
  /// South African augmentation system
  ASBAS,
  /// Autralia / NZ augmentation system
  SPAN,
  /// SBAS is used to describe SBAS (augmentation)
  /// vehicles without much more information
  SBAS,
  /// Australia-NZ Geoscience system
  AusNZ,
  /// Group Based SBAS
  GBAS,
  /// Nigerian SBAS
  NSAS,
  /// Algerian SBAS
  ASAL,
  /// `Mixed` for Mixed constellations
  /// RINEX files description
  Mixed,
};

struct Constellation {
  constexpr Constellation() : id(ConstellationEnum::GPS) {}
  constexpr Constellation(ConstellationEnum id) : id(id) {}
  static Constellation form_str(std::string_view str) {
    std::string s(str);
    boost::algorithm::trim(s);
    boost::algorithm::to_lower(s);
    ConstellationEnum result;
    if (s == "g" || s == "gps")
      result = ConstellationEnum::GPS;
    else if (s == "c" || s == "bds")
      result = ConstellationEnum::BeiDou;
    else if (s == "e" || s == "gal")
      result = ConstellationEnum::Galileo;
    else if (s == "r" || s == "glo")
      result = ConstellationEnum::Glonass;
    else if (s == "j" || s == "qzss")
      result = ConstellationEnum::QZSS;
    else if (s == "i" || s == "irnss")
      result = ConstellationEnum::IRNSS;
    else if (s == "s" || s == "sbas")
      result = ConstellationEnum::SBAS;
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
      result = ConstellationEnum::Glonass;
    else if (s.find("beidou") != std::string::npos)
      result = ConstellationEnum::BeiDou;
    else if (s.find("galileo") != std::string::npos)
      result = ConstellationEnum::Galileo;
    else if (s.find("qzss") != std::string::npos)
      result = ConstellationEnum::QZSS;
    else if (s.find("sbas") != std::string::npos || s.find("geo") != std::string::npos)
      result = ConstellationEnum::SBAS;
    else if (s.find("irnss") != std::string::npos || s.find("navic") != std::string::npos)
      result = ConstellationEnum::IRNSS;
    else if (s.find("mix") != std::string::npos)
      result = ConstellationEnum::Mixed;
    else {
      auto error_msg = std::format("Unable to parse string \"{}\" to Constellation", s);
      nav_error(error_msg);
      throw std::runtime_error(error_msg);
    }

    return Constellation(result);
  }
  bool is_sbas() const {
    return id == ConstellationEnum::WAAS || id == ConstellationEnum::EGNOS ||
           id == ConstellationEnum::MSAS || id == ConstellationEnum::GAGAN ||
           id == ConstellationEnum::BDSBAS || id == ConstellationEnum::KASS ||
           id == ConstellationEnum::SDCM || id == ConstellationEnum::ASBAS ||
           id == ConstellationEnum::SPAN || id == ConstellationEnum::SBAS ||
           id == ConstellationEnum::AusNZ || id == ConstellationEnum::GBAS ||
           id == ConstellationEnum::NSAS || id == ConstellationEnum::ASAL;
  }
  bool is_mixed() const { return id == ConstellationEnum::Mixed; }
  constexpr std::string_view name() const { return magic_enum::enum_name(id); }
  constexpr bool operator==(const Constellation& rhs) const { return this->id == rhs.id; }
  constexpr bool operator!=(const Constellation& rhs) const { return this->id != rhs.id; }
  ConstellationEnum id;
};

struct Sv {
  Sv(u8 prn, ConstellationEnum cons_enum) : prn(prn), constellation(cons_enum) {}
  Sv(u8 prn, Constellation cons) : prn(prn), constellation(cons) {}
  Sv(ConstellationEnum cons_enum) : prn(0), constellation(cons_enum) {}
  Sv(Constellation cons) : prn(0), constellation(cons) {}
  static Sv from_str(std::string_view str) {
    std::string_view first(str.data(), 1);
    auto constellation = Constellation::form_str(first);
    u8 prn;
    auto [ptr, ec] = std::from_chars(str.data() + 1, str.data() + str.size(), prn);
    if (ec != std::errc()) {
      auto error_msg =
          std::format("Unable to parse string \"{}\" to Sv", std::string(str.data() + 1));
      nav_error(error_msg);
      throw std::runtime_error(error_msg);
    }
    return Sv{prn, constellation};
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
      auto error_msg = "Cannot compare prn of Sv with different systems.";
      nav_error(error_msg);
      throw std::runtime_error(error_msg);
    }
    return prn <=> rhs.prn;
  }

  u8 prn;
  Constellation constellation;
};

}  // namespace navp

template <>
struct std::formatter<navp::Constellation, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <class FmtContext>
  FmtContext::iterator format(navp::Constellation cons, FmtContext& ctx) const {
    using navp::ConstellationEnum;
    switch (cons.id) {
      case ConstellationEnum::GPS:
        return std::format_to(ctx.out(), "G");
      case ConstellationEnum::Glonass:
        return std::format_to(ctx.out(), "R");
      case ConstellationEnum::Galileo:
        return std::format_to(ctx.out(), "E");
      case ConstellationEnum::BeiDou:
        return std::format_to(ctx.out(), "C");
      case ConstellationEnum::QZSS:
        return std::format_to(ctx.out(), "J");
      case ConstellationEnum::IRNSS:
        return std::format_to(ctx.out(), "I");
      default: {
        if (cons.is_sbas()) {
          return std::format_to(ctx.out(), "S");
        } else if (cons.is_mixed()) {
          return std::format_to(ctx.out(), "M");
        } else {
          throw std::format("Format Constellation {} error", static_cast<navp::u8>(cons.id));
        }
      }
    }
  }
};

template <>
struct std::formatter<navp::Sv, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }
  template <class FormatContext>
  auto format(navp::Sv sv, FormatContext& ctx) const {
    return std::format_to(ctx.out(), "{}{:02}", sv.constellation, sv.prn);
  }
};

template <>
struct std::formatter<navp::Carrier, char> {
  template <class ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <class FormatContext>
  auto format(navp::Carrier carrier, FormatContext& ctx) const {
    try {
      const std::string& carrier_str = navp::constants::CARRIER_TABLE.right.at(carrier.id);
      return std::format_to(ctx.out(), "{}", carrier_str);
    } catch (const std::out_of_range&) {
      return std::format_to(ctx.out(), "Unknown Carrier");
    }
  }
};