#pragma once

#include <boost/algorithm/string.hpp>

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

  static Result<FilterOperand, FilterParseOperatorError> from_str(const char* str) {
    std::string s(str);
    boost::algorithm::trim(s);
    if (s.starts_with(">=")) {
      return FilterOperand{CompareOperatorEnum::GreaterEqual};
    } else if (s.starts_with(">")) {
      return FilterOperand{CompareOperatorEnum::Greater};
    } else if (s.starts_with("<=")) {
      return FilterOperand{CompareOperatorEnum::LessEqual};
    } else if (s.starts_with("<")) {
      return FilterOperand{CompareOperatorEnum::Less};
    } else if (s.starts_with("!=")) {
      return FilterOperand{CompareOperatorEnum::NotEqual};
    } else if (s.starts_with("=")) {
      return FilterOperand{CompareOperatorEnum::Equal};
    } else {
      return FilterParseOperatorError(std::format("Can't parse {} into \'FilterOperand\'", str));
    }
  }
};

using EpochItem = navp::EpochUtc;                                           /// ID 0
using CarrierItem = std::vector<navp::sensors::gnss::Carrier>;              /// ID 1
using ConstellationItem = std::vector<navp::sensors::gnss::Constellation>;  /// ID 2
using SvItem = std::vector<navp::sensors::gnss::Sv>;                        /// ID 3
using SnrItem = f64;                                                        /// ID 4
using ElevationItem = f64;                                                  /// ID 5
using AzimuthItem = f64;                                                    /// ID 6
using ComplexItem = std::vector<std::string>;                               /// ID 7

class NAVP_EXPORT FilterItems : public std::variant<EpochItem, CarrierItem, ConstellationItem, SvItem, SnrItem,
                                                    ElevationItem, AzimuthItem, ComplexItem> {
 public:
  // todo
  static Result<FilterItems, FilterParseItemError> from_str(const char* str) {
    /*
     * type guessing
     */
    std::string s(str);
    boost::algorithm::trim(s);
    std::vector<std::string> items;
    static auto is_comma = [](char c) { return c == 'c'; };
    boost::algorithm::split(items, s, is_comma);
    /*
     * Epoch
     */
    auto epoch_res = EpochUtc::from_str("%Y-%m-%d %H:%M:%S", items[0].c_str());
    if (epoch_res.is_ok()) {
      return Ok(FilterItems(epoch_res.unwrap_unchecked()));
    }
    /*
     * Carrier
     */
    if (Carrier::from_str(items[0].c_str()).is_ok()) {
      std::vector<Carrier> carrier_res(items.size());
      for (u16 i = 0; i < items.size(); ++i) {
        auto carrier = Carrier::from_str(items[i].c_str());
        carrier_res[i] = carrier.unwrap();
      }
      return Ok(FilterItems(carrier_res));
    }
    /*
     * Constellation
     */
    if (Constellation::form_str(items[0].c_str()).is_ok()) {
      std::vector<Constellation> constellation_res(items.size());
      for (u16 i = 0; i < items.size(); ++i) {
        auto constellation = Constellation::form_str(items[i].c_str());
        constellation_res[i] = constellation.unwrap();
      }
      return Ok(FilterItems(constellation_res));
    }
    /*
     * Sv
     */
    if (Sv::from_str(items[0].c_str()).is_ok()) {
      std::vector<Sv> sv_res(items.size());
      for (u16 i = 0; i < items.size(); ++i) {
        auto sv = Sv::from_str(items[i].c_str());
        sv_res[i] = sv.unwrap();
      }
      return Ok(FilterItems(sv_res));
    }
    return Ok(FilterItems(items));
  }
};

}  // namespace navp::filter