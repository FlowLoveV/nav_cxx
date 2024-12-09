#pragma once

#include "filter/items.hpp"
#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/time.hpp"
#include "utils/macro.hpp"

namespace navp::filter {

using navp::Epoch;
using navp::sensors::gnss::Carrier;
using navp::sensors::gnss::Sv;

struct NAVP_EXPORT MaskFilter {
  CompareOperatorEnum op;
  FilterItems item;
};

template <typename Self>
void NAVP_EXPORT mask_filter(Self& self, const MaskFilter& filter) noexcept;
template <typename Self>
Self NAVP_EXPORT mask_filter(const Self& self, const MaskFilter& filter) noexcept;

}  // namespace navp::filter