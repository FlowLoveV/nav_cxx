#pragma once

#include "utils/attitude.hpp"
#include "utils/macro.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"
#include "utils/types.hpp"

namespace navp::solution {

using utils::Attitude;
using utils::Coordinate;
using utils::NavMatrix33f64;

using utils::XYZ;

enum class NAVP_EXPORT SolutionMode : u8 {
  NONE = 0,
  FIXED = 1,
  FLOAT = 2,
  SBAS = 3,
  DGPS = 4,
  SINGLE = 5,
  PPP = 6,
  DR_LOOSE = 7,
  DR_TIGHT = 8,
};

struct NAVP_EXPORT PvtSolution {
  EpochUtc time;
  Coordinate<XYZ> position, velicity; /* position/velocity (m|m/s) */
  f32 qr[6];                          /* position variance/covariance (m^2) */
  f32 qv[6];                          /* velocity variance/covariance (m^2/s^2) */
  f64 dtr[6];                         /* receiver clock bias to time systems (s) */
  SolutionMode mode = SolutionMode::NONE;
  u8 type;   /* type (0:xyz-ecef,1:enu-baseline) */
  u8 ns;     /* number of valid satellites */
  f32 age;   /* age of differential (s) */
  f32 ratio; /* AR ratio factor for valiation */
  f32 thres; /* AR ratio threshold for valiation */
};

}  // namespace navp::solution