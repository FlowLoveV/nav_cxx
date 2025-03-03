#pragma once

#include "sensors/gnss/analysis.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

class CycleSlip;

class CycleSlip {
  EpochUtc time;
};

}  // namespace navp::sensors::gnss