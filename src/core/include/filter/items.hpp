#pragma once

#include <boost/algorithm/string.hpp>

#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/time.hpp"
#include "utils/types.hpp"

namespace navp::filter {

using sensors::gnss::Carrier;
using sensors::gnss::Constellation;
using sensors::gnss::Sv;

enum class CompareOperatorEnum : u8 {
  Greater,
  Less,
  Equal,
  GreaterEqual,
  LessEqual,
  NotEqual,
};

struct FilterOperand {
  CompareOperatorEnum op;

  static NavResult<FilterOperand> from_str(const char* str) {
    std::string s(str);
    boost::algorithm::trim(s);
    if (s.starts_with(">=")) {
      return Ok(CompareOperatorEnum::GreaterEqual);
    } else if (s.starts_with(">")) {
      return Ok(CompareOperatorEnum::Greater);
    } else if (s.starts_with("<=")) {
      return Ok(CompareOperatorEnum::LessEqual);
    } else if (s.starts_with("<")) {
      return Ok(CompareOperatorEnum::Less);
    } else if (s.starts_with("!=")) {
      return Ok(CompareOperatorEnum::NotEqual);
    } else if (s.starts_with("=")) {
      return Ok(CompareOperatorEnum::Equal);
    } else {
      return Err(errors::NavError::Utils::Filter::ParseOperandError);
    }
  }
};

using EpochItem = navp::Epoch<UTC>;                                         /// ID 0
using CarrierItem = std::vector<navp::sensors::gnss::Carrier>;              /// ID 1
using ConstellationItem = std::vector<navp::sensors::gnss::Constellation>;  /// ID 2
using SvItem = std::vector<navp::sensors::gnss::Sv>;                        /// ID 3
using SnrItem = f64;                                                        /// ID 4
using ElevationItem = f64;                                                  /// ID 5
using AzimuthItem = f64;                                                    /// ID 6
using ComplexItem = std::vector<std::string>;                               /// ID 7

class FilterItems : public std::variant<EpochItem, CarrierItem, ConstellationItem, SvItem, SnrItem, ElevationItem,
                                        AzimuthItem, ComplexItem> {
 public:
  static NavResult<FilterItems> from_str(const char* str) {
    /*
     * type guessing
     */
    std::string s(str);
    boost::algorithm::trim(s);
    std::vector<std::string> items;
    static auto is_comma = [](char c) {return c == 'c';};
    boost::algorithm::split(items, s, is_comma);
    /*
     * Epoch
     */
    auto epoch_res = Epoch<UTC>::from_str("%Y-%m-%d %H:%M:%S", items[0].c_str());
    if (epoch_res.is_ok()) {
      return Ok(FilterItems(epoch_res.unwrap()));
    }
    /*
     * Carrier
     */
    if (Carrier::from_str(items[0].c_str()).is_ok()) {
      std::vector<Carrier> carrier_res(items.size());
      for (const auto& item : items) {
        auto carrier = Carrier::from_str(item.c_str());
        carrier_res.emplace_back(carrier.unwrap());
        return Ok(FilterItems(carrier_res));
      }
    }
    /*
     * Constellation
     */
    if (Constellation::form_str(items[0].c_str()).is_ok()) {
      std::vector<Constellation> constellation_res(items.size());
      for (const auto& item : items) {
        auto constellation = Constellation::form_str(item.c_str());
        constellation_res.emplace_back(constellation.unwrap());
        return Ok(FilterItems(constellation_res));
      }
    }
    /*
     * Sv
     */
    if (Sv::from_str(items[0].c_str()).is_ok()) {
      std::vector<Sv> sv_res(items.size());
      for (const auto& item : items) {
        auto sv = Sv::from_str(item.c_str());
        sv_res.emplace_back(sv.unwrap());
        return Ok(FilterItems(sv_res));
      }
    }
    return Ok(FilterItems(items));
  }
};


}  // namespace navp::filter