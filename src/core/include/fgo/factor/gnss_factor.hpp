#pragma once

#include "ceres/ceres.h"
#include "sensors/gnss/observation.hpp"

namespace navp::fgo {

// forward declation


class GnssFactor;

class GnssFactor : ceres::SizedCostFunction<6, 21>{
 public:
    
 protected:
};

}  // namespace navp::fgo