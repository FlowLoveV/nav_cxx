#pragma once

#include "io/record.hpp"
#include "utils/macro.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"
#include "utils/types.hpp"

namespace navp::solution {

enum class NAVP_EXPORT SolutionModeEnum : u8 {
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

struct NAVP_EXPORT PvtSolutionRecord : public io::Record {
  virtual ~PvtSolutionRecord() = default;

  EpochUtc time;
  utils::CoordinateXyz position, velocity; /* position/velocity (m|m/s) */
  utils::CoordinateBlh blh;                /* latitude/longitude/height (rad|rad|m) */
  f32 qr[6];                               /* position variance/covariance xx,xy,xz,yy,yz,zz (m^2) */
  f32 qv[6];                               /* velocity variance/covariance xx,xy,xz,yy,yz,zz (m^2/s^2) */
  f64 sigma_r, sigma_v;                    /* position/velocity sigma */
  f64 dtr[6];                              /* receiver clock bias to time systems (s)
                                              [0] = BDS, [1] = GPS
                                              ... to be determined
                                              [5] = receiver clock speed
                                            */
  SolutionModeEnum mode = SolutionModeEnum::NONE;
  u8 type; /* type (0:xyz-ecef,1:enu-baseline) */
  u8 syss; /* number of systems */
  u8 ns;   /* number of valid satellites */
};

}  // namespace navp::solution