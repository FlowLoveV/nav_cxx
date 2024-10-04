#include <cstring>

#include "utils/macro.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "utils/angle.hpp"
#include "utils/space.hpp"

using namespace navp;
using namespace navp::utils;
#define FE_WGS84 (1.0 / 298.257223563) /* earth flattening (WGS84) */
#define RE_WGS84 6378137.0             /* earth semimajor axis (WGS84) (m) */
#define PI 3.1415926535897932          /* pi */

f64 dot(const f64* a, const f64* b, int n) {
  f64 c = 0.0;
  while (--n >= 0) c += a[n] * b[n];
  return c;
}

void ecef2pos(const f64* r, f64* pos) {
  f64 e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

  for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
    zk = z;
    sinp = z / sqrt(r2 + z * z);
    v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
    z = r[2] + v * e2 * sinp;
  }
  pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
  pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
  pos[2] = sqrt(r2 + z * z) - v;
}

void xyz2enu(const f64* pos, f64* E) {
  f64 sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

  E[0] = -sinl;
  E[3] = cosl;
  E[6] = 0.0;
  E[1] = -sinp * cosl;
  E[4] = -sinp * sinl;
  E[7] = cosp;
  E[2] = cosp * cosl;
  E[5] = cosp * sinl;
  E[8] = sinp;
}

void pos2ecef(const f64* pos, f64* r) {
  f64 sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
  f64 e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

  r[0] = (v + pos[2]) * cosp * cosl;
  r[1] = (v + pos[2]) * cosp * sinl;
  r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}

void matmul(const char* tr, int n, int k, int m, f64 alpha, const f64* A, const f64* B, f64 beta, f64* C) {
  f64 d;
  int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

  for (i = 0; i < n; i++)
    for (j = 0; j < k; j++) {
      d = 0.0;
      switch (f) {
        case 1:
          for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m];
          break;
        case 2:
          for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k];
          break;
        case 3:
          for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m];
          break;
        case 4:
          for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k];
          break;
      }
      if (beta == 0.0)
        C[i + j * n] = alpha * d;
      else
        C[i + j * n] = alpha * d + beta * C[i + j * n];
    }
}

void ecef2enu(const f64* pos, const f64* r, f64* e) {
  f64 E[9];

  xyz2enu(pos, E);
  matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

TEST_CASE("crs2crs") {
  Coordinate<BLH> blh{to_radians(30.13145), to_radians(114.13145), 20};
  Coordinate<BLH> blh1{to_radians(30.131456), to_radians(114.131456), 19};
  // blh->xyz
  auto xyz = blh.to_xyz();
  f64 p_xyz[3];
  pos2ecef(blh.data(), p_xyz);
  CHECK(memcmp(xyz.data(), p_xyz, 24) == 0);
  // xyz->blh
  f64 p_blh[3];
  ecef2pos(p_xyz, p_blh);
  CHECK(memcmp(p_blh, xyz.to_blh().data(), 24) == 0);
  // blh->enu
  f64 r[3], e[3];
  memcpy(r, (blh1.to_xyz() - blh.to_xyz()).data(), 24);
  ecef2enu(blh.data(), r, e);
  auto enu = blh.to_enu(blh1);
  CHECK(memcmp(e, enu.data(), 24) == 0);
}
