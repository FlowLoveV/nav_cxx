#pragma once

#include "utils/attitude.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"
#include "utils/types.hpp"

namespace navp::solution {

using utils::Attitude;
using utils::Coordinate;
using utils::NavMatrix33f64;
using utils::XYZ;

enum class PositioningMode : u8 {
  SPP = 0,
  RTK = 1,
  PPP = 2,
  LC = 3,
  TC = 4,
  FGO = 5,
};

struct PositioningResult {
  Epoch<UTC> time;
  Coordinate<XYZ> position;
  Coordinate<XYZ> velicity;
  Attitude attitude;
  NavMatrix33f64 pos_var;
  NavMatrix33f64 vel_var;
  NavMatrix33f64 att_var;
  PositioningMode mode;
  //   void write(std::istream& os) const noexcept;
};

}  // namespace navp::solution