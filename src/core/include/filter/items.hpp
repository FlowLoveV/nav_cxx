#pragma once

#include "filter/filter_exception.hpp"
#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/macro.hpp"
#include "utils/time.hpp"
#include "utils/types.hpp"

namespace navp::filter {

using sensors::gnss::Carrier;
using sensors::gnss::Constellation;
using sensors::gnss::Sv;

class NavFilterError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class NAVP_EXPORT CompareOperatorEnum : u8 {
  Greater,
  Less,
  Equal,
  GreaterEqual,
  LessEqual,
  NotEqual,
};

struct NAVP_EXPORT FilterOperand {
  CompareOperatorEnum op;

  static auto from_str(std::string_view str) -> Result<FilterOperand, FilterParseOperatorError>;

  size_t size() const noexcept;

  template <typename _CompareT>
  bool apply(const _CompareT& lhs, const _CompareT& rhs) const {
    switch (op) {
      case CompareOperatorEnum::Greater:
        return lhs > rhs;
      case CompareOperatorEnum::Less:
        return lhs < rhs;
      case CompareOperatorEnum::Equal:
        return lhs == rhs;
      case CompareOperatorEnum::NotEqual:
        return lhs != rhs;
      case CompareOperatorEnum::GreaterEqual:
        return lhs >= rhs;
      case CompareOperatorEnum::LessEqual:
        return lhs <= rhs;
      default:
        throw NavFilterError("Unknown compare operator!");
    }
  }
};

using EpochItem = navp::EpochUtc;
using CarrierItem = navp::sensors::gnss::Carrier;
using CodeItem = navp::sensors::gnss::ObsCodeEnum;
using ConstellationItem = navp::sensors::gnss::Constellation;
using SvItem = navp::sensors::gnss::Sv;
struct SnrItem {
  f64 val;

  constexpr auto operator<=>(const SnrItem&) const = default;
};
struct ElevationItem {
  f64 val;

  constexpr auto operator<=>(const ElevationItem&) const = default;
};
struct AzimuthItem {
  f64 val;

  constexpr auto operator<=>(const AzimuthItem&) const = default;
};

class NAVP_EXPORT FilterItem
    : public std::variant<EpochItem, CarrierItem, ConstellationItem, SvItem, SnrItem, ElevationItem, AzimuthItem> {
 public:
  using base_type =
      std::variant<EpochItem, CarrierItem, ConstellationItem, SvItem, SnrItem, ElevationItem, AzimuthItem>;

  using base_type::variant;

  static auto from_str(std::string_view str) -> Result<FilterItem, FilterParseItemError>;

  bool matched_with(const FilterItem& item) const noexcept;
};

}  // namespace navp::filter