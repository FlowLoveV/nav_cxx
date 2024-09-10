#pragma once

#include "mode.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "spp_config.hpp"
namespace navp::solution {

using navp::sensors::gnss::RecordGnssNav;
using navp::sensors::gnss::RecordGnssObs;

class SolutionSpp {
  SppConfig config;
  std::forward_list<RecordGnssNav> nav;
  RecordGnssObs obs;
  mutable std::forward_list<PositioningResult> result;
};

}  // namespace navp::solution