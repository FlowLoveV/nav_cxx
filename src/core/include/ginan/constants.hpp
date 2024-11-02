#pragma once

#include <map>

#include "sensors/gnss/enums.hpp"

namespace navp::ginan {

using sensors::gnss::ConstellationEnum;
using sensors::gnss::FreTypeEnum;
using sensors::gnss::ObsCodeEnum;

extern std::map<FreTypeEnum, f64> genericWavelength;
extern std::map<ConstellationEnum, std::map<ObsCodeEnum, FreTypeEnum>> code2Freq;
extern std::map<FreTypeEnum, f64> roughFrequency;

}  // namespace navp::ginan