#include "filter/filter.hpp"

#include <boost/algorithm/string.hpp>

#include "utils/angle.hpp"
#include "utils/string_utils.hpp"

namespace navp::filter {

Result<FilterOperand, FilterParseOperatorError> FilterOperand::from_str(std::string_view str) {
  if (str.starts_with(">=")) {
    return FilterOperand{CompareOperatorEnum::GreaterEqual};
  } else if (str.starts_with(">")) {
    return FilterOperand{CompareOperatorEnum::Greater};
  } else if (str.starts_with("<=")) {
    return FilterOperand{CompareOperatorEnum::LessEqual};
  } else if (str.starts_with("<")) {
    return FilterOperand{CompareOperatorEnum::Less};
  } else if (str.starts_with("!=")) {
    return FilterOperand{CompareOperatorEnum::NotEqual};
  } else if (str.starts_with("=")) {
    return FilterOperand{CompareOperatorEnum::Equal};
  } else {
    return FilterParseOperatorError(std::format("Can't parse \'{}\' into \'FilterOperand\'!", str));
  }
}

size_t FilterOperand::size() const noexcept {
  switch (op) {
    case CompareOperatorEnum::Greater:
      return 1;
    case CompareOperatorEnum::Less:
      return 1;
    case CompareOperatorEnum::Equal:
      return 1;
    case CompareOperatorEnum::NotEqual:
      return 2;
    case CompareOperatorEnum::GreaterEqual:
      return 2;
    case CompareOperatorEnum::LessEqual:
      return 2;
    default:
      return 0;
  }
}

Result<FilterItem, FilterParseItemError> FilterItem::from_str(std::string_view str) {
  // Epoch
  if (auto epoch_res = EpochUtc::from_str("%Y-%m-%d %H:%M:%S", str.data()); epoch_res.is_ok()) {
    return Ok(FilterItem(epoch_res.unwrap_unchecked()));
  }
  // Carrier
  if (auto carrier_res = Carrier::from_str(str.data()); carrier_res.is_ok()) {
    return Ok(FilterItem(carrier_res.unwrap_unchecked()));
  }
  // Constellation
  if (auto constellation_res = Constellation::form_str(str.data()); constellation_res.is_ok()) {
    return Ok(FilterItem(constellation_res.unwrap_unchecked()));
  }
  // sv
  if (auto sv_res = Sv::from_str(str.data()); sv_res.is_ok()) {
    return Ok(FilterItem(sv_res.unwrap_unchecked()));
  }
  auto _type_char = str.back();
  // snr
  if (_type_char == 's' || _type_char == 'S') {
    f64 val = 0;
    if (auto [p, ec] = std::from_chars(str.data(), str.data() + str.size() - 1, val); ec == std::errc()) {
      return Ok(SnrItem{val});
    }
  }
  // elevation
  if (_type_char == 'e' || _type_char == 'E') {
    f64 val = 0;
    if (auto [p, ec] = std::from_chars(str.data(), str.data() + str.size() - 1, val); ec == std::errc()) {
      return Ok(ElevationItem{to_radians(val)});
    }
  }
  // azimuth
  if (_type_char == 'a' || _type_char == 'A') {
    f64 val = 0;
    if (auto [p, ec] = std::from_chars(str.data(), str.data() + str.size() - 1, val); ec == std::errc()) {
      return Ok(AzimuthItem{to_radians(val)});
    }
  }
  return Err(FilterParseItemError(std::format("Can't parse \'{}\' into \'FilterItem\'!", str)));
}

bool FilterItem::matched_with(const FilterItem& item) const noexcept { return index() == item.index(); }

auto Filter::from_str(std::string_view str) -> Result<Filter, FilterParseError> {
  utils::trim_left(str);
  auto op = FilterOperand::from_str(str).unwrap_throw();
  auto filter_res = FilterItem::from_str(str.substr(op.size())).unwrap_throw();
  return Ok(Filter{op.op, filter_res});
}

auto Filter::apply(const FilterItem& item) const noexcept -> bool {
  if (__item.matched_with(item)) {
    switch (item.index()) {
      case 0:
        return __op.apply(std::get<EpochItem>(item), std::get<EpochItem>(__item));
      case 1:
        return __op.apply(std::get<CarrierItem>(item), std::get<CarrierItem>(__item));
      case 2:
        return __op.apply(std::get<ConstellationItem>(item), std::get<ConstellationItem>(__item));
      case 3:
        return __op.apply(std::get<SvItem>(item), std::get<SvItem>(__item));
      case 4:
        return __op.apply(std::get<SnrItem>(item), std::get<SnrItem>(__item));
      case 5:
        return __op.apply(std::get<ElevationItem>(item), std::get<ElevationItem>(__item));
      case 6:
        return __op.apply(std::get<AzimuthItem>(item), std::get<AzimuthItem>(__item));
      default:
        return true;
    }
    // if (std::holds_alternative<EpochItem>(item)) {
    //   return __op.apply(std::get<EpochItem>(item), std::get<EpochItem>(__item));
    // }
    // if (std::holds_alternative<CarrierItem>(item)) {
    //   return __op.apply(std::get<CarrierItem>(item), std::get<CarrierItem>(__item));
    // }
    // if (std::holds_alternative<ConstellationItem>(item)) {
    //   return __op.apply(std::get<ConstellationItem>(item), std::get<ConstellationItem>(__item));
    // }
    // if (std::holds_alternative<SvItem>(item)) {
    //   return __op.apply(std::get<SvItem>(item), std::get<SvItem>(__item));
    // }
    // if (std::holds_alternative<SnrItem>(item)) {
    //   return __op.apply(std::get<SnrItem>(item), std::get<SnrItem>(__item));
    // }
    // if (std::holds_alternative<ElevationItem>(item)) {
    //   return __op.apply(std::get<ElevationItem>(item), std::get<ElevationItem>(__item));
    // }
    // if (std::holds_alternative<AzimuthItem>(item)) {
    //   return __op.apply(std::get<AzimuthItem>(item), std::get<AzimuthItem>(__item));
    // }
  }
  return true;
}

auto MaskFilters::from_str(std::vector<std::string_view> str) -> Result<MaskFilters, FilterParseError> {
  std::vector<Filter> filters;
  filters.reserve(str.size());
  for (const auto& item : str) {
    filters.emplace_back(Filter::from_str(item).unwrap_throw());
  }
  return Ok(MaskFilters{.__filter = std::move(filters)});
}

Result<MaskFilters, FilterParseError> MaskFilters::from_str(std::string_view str) {
  std::vector<Filter> filters;
  std::string s(str);
  boost::algorithm::trim(s);
  std::vector<std::string> items;
  boost::algorithm::split(items, s, boost::algorithm::is_any_of(","));
  filters.reserve(items.size());
  for (const auto& item : items) {
    if (item.empty()) continue;
    filters.emplace_back(Filter::from_str(item).unwrap_throw());
  }
  return Ok(MaskFilters{.__filter = std::move(filters)});
}

auto MaskFilters::apply(const FilterItem& item) const noexcept -> bool {
  for (const auto& filter : __filter) {
    if (!filter.apply(item)) return false;
  }
  return true;
}

}  // namespace navp::filter