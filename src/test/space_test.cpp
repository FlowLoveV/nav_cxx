#include <cstring>

#include "utils/macro.hpp"
#include "rtklib.h"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "utils/angle.hpp"
#include "utils/space.hpp"

using namespace navp;
using namespace navp::details;

TEST_CASE("crs2crs") {
  Coordinate<BLH> blh{to_radians(30.13145), to_radians(114.13145), 20};
  Coordinate<BLH> blh1{to_radians(30.131456), to_radians(114.131456), 19};
  // blh->xyz
  auto xyz = blh.to_xyz();
  double p_xyz[3];
  pos2ecef(blh.data(), p_xyz);
  CHECK(memcmp(xyz.data(), p_xyz, 24) == 0);
  // xyz->blh
  double p_blh[3];
  ecef2pos(p_xyz, p_blh);
  CHECK(memcmp(p_blh, xyz.to_blh().data(), 24) == 0);
  // blh->enu
  double r[3], e[3];
  memcpy(r, (blh1.to_xyz() - blh.to_xyz()).data(), 24);
  ecef2enu(blh.data(), r, e);
  auto enu = blh.to_enu(blh1);
  CHECK(memcmp(e, enu.data(), 24) == 0);
}
