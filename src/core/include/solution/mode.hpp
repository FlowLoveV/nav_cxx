#pragma once

#include "utils/types.hpp"
#include "utils/attitude.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"

namespace navp::solution {

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
  Matrix3d pos_var;
  Matrix3d vel_var;
  Matrix3d att_var;
  PositioningMode mode;
  //   void write(std::istream& os) const noexcept;
};

}  // namespace navp::solution