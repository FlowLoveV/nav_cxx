#pragma once

#include "utils/types.hpp"

namespace navp::algorithm {

enum class AlgorithmEnum : u8 {
  WeightedLeastSquare = 0,
  KalmanFilter = 1,
  FactorGraphOptimization = 2,
};

} 