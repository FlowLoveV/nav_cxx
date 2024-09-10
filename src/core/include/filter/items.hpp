#pragma once

#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/types.hpp"
#include "utils/time.hpp"

namespace navp::filter {

enum class CompareOperatorEnum : u8 {
  Greater,
  Less,
  Equal,
  GreaterEqual,
  LessEqual,
  NotEqual,
};


using EpochItem = navp::Epoch<UTC>;
using CarrierItem = navp::sensors::gnss::Carrier;
using ConstellationItem = navp::sensors::gnss::Constellation;
using SvItem = navp::sensors::gnss::Sv;
using SnrItem = f64;
using ElevationItem = f64;
using AzimuthItem = f64;


}  // namespace navp::filter