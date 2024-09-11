#pragma once


#include "items.hpp"
#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/time.hpp"


namespace navp::filter {

using navp::Epoch;
using navp::sensors::gnss::Carrier;
using navp::sensors::gnss::Sv;

// clang-format off
// typedef std::variant<MaskFilter<Sv>, 
//                     MaskFilter<Carrier>,
//                     MaskFilter<Epoch<UTC>>,
//                     std::monostate> FilterGnssItems;
// clang-format on


}  // namespace navp::filter