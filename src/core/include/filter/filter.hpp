#pragma once

#include "items.hpp"
#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/time.hpp"

namespace navp::filter {

using navp::Epoch;
using navp::sensors::gnss::Carrier;
using navp::sensors::gnss::Sv;

struct MaskFilter {
  CompareOperatorEnum op;
  FilterItems item;
};

template <typename Self>
void mask_filter(Self& self, const MaskFilter& filter) noexcept;
template <typename Self>
Self mask_filter(const Self& self, const MaskFilter& filter) noexcept;

}  // namespace navp::filter