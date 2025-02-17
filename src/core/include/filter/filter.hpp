#pragma once

#include "filter/items.hpp"
#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/macro.hpp"
#include "utils/time.hpp"

namespace navp::filter {

using navp::Epoch; 
using navp::sensors::gnss::Carrier;
using navp::sensors::gnss::Sv;

struct NAVP_EXPORT Filter {
  FilterOperand __op;
  FilterItem __item;

  static auto from_str(std::string_view str) -> Result<Filter, FilterParseError>;

  auto apply(const FilterItem& item) const noexcept -> bool;
};

struct NAVP_EXPORT MaskFilters {
  static auto from_str(std::vector<std::string_view> str) -> Result<MaskFilters, FilterParseError>;
  static auto from_str(std::string_view str) -> Result<MaskFilters, FilterParseError>;

  auto apply(const FilterItem& item) const noexcept -> bool;

  std::vector<Filter> __filter;
};

}  // namespace navp::filter