#include "utils/space.hpp"

// #include "rtklib.h"
#define FE_WGS84 (1.0 / 298.257223563) /* earth flattening (WGS84) */
#define RE_WGS84 6378137.0             /* earth semimajor axis (WGS84) (m) */
#define PI 3.1415926535897932          /* pi */

double dot(const double* a, const double* b, int n) {
  double c = 0.0;
  while (--n >= 0) c += a[n] * b[n];
  return c;
}

void ecef2pos(const double* r, double* pos) {
  double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

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

void xyz2enu(const double* pos, double* E) {
  double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

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

void pos2ecef(const double* pos, double* r) {
  double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
  double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

  r[0] = (v + pos[2]) * cosp * cosl;
  r[1] = (v + pos[2]) * cosp * sinl;
  r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}

namespace navp {

auto Coordinate<XYZ>::to_blh() const noexcept -> Coordinate<BLH> {
  Coordinate<BLH> blh;
  ecef2pos(this->data(), blh.data());
  return blh;
}

auto Coordinate<XYZ>::to_enu_matrix() const noexcept -> Matrix3d {
  Matrix3d e;
  xyz2enu(this->to_blh().data(), e.data());
  return e;
}

auto Coordinate<XYZ>::to_enu(const Coordinate& xyz) const noexcept -> Coordinate<ENU> {
  Coordinate<ENU> enu;
  Matrix3d e;
  Vector3d r = xyz - *this;
  xyz2enu(this->to_blh().data(), e.data());
  return e * r;
}

auto Coordinate<XYZ>::to_enu(const std::span<Coordinate>& span) const noexcept -> std::vector<Coordinate<ENU>> {
  auto e = this->to_enu_matrix();
  std::vector<Coordinate<ENU>> res(span.size());
  for (const auto& xyz : span) {
    Vector3d r = xyz - *this;
    res.emplace_back(e * r);
  }
  return res;
}

auto Coordinate<XYZ>::to_enu(const Coordinate<BLH>& blh) const noexcept -> Coordinate<ENU> {
  return this->to_enu(blh.to_xyz());
}

auto Coordinate<XYZ>::to_enu(const std::span<Coordinate<BLH>>& span) const noexcept -> std::vector<Coordinate<ENU>> {
  auto e = this->to_enu_matrix();
  std::vector<Coordinate<ENU>> res(span.size());
  for (const auto& blh : span) {
    Vector3d r = blh.to_xyz() - *this;
    res.emplace_back(e * r);
  }
  return res;
}

auto Coordinate<XYZ>::conv_enu(const Matrix3d& conv_xyz) const -> Matrix3d {
  auto e = this->to_enu_matrix();
  return e * conv_xyz * e.transpose();
}

auto Coordinate<BLH>::to_xyz() const noexcept -> Coordinate<XYZ> {
  Coordinate<XYZ> xyz;
  pos2ecef(this->data(), xyz.data());
  return xyz;
}

auto Coordinate<BLH>::to_enu_matrix() const noexcept -> Matrix3d {
  Matrix3d e;
  xyz2enu(this->data(), e.data());
  return e;
}

auto Coordinate<BLH>::to_enu(const Coordinate& blh) const noexcept -> Coordinate<ENU> {
  Vector3d r = blh.to_xyz() - this->to_xyz();
  auto e = this->to_enu_matrix();
  return e * r;
}

auto Coordinate<BLH>::to_enu(const std::span<Coordinate>& span) const noexcept -> std::vector<Coordinate<ENU>> {
  Matrix3d e = this->to_enu_matrix();
  auto xyz0 = this->to_xyz();
  std::vector<Coordinate<ENU>> res(span.size());
  for (const auto& blh : span) {
    Vector3d r = blh.to_xyz() - xyz0;
    res.emplace_back(e * r);
  }
  return res;
}

auto Coordinate<BLH>::to_enu(const Coordinate<XYZ>& xyz) const noexcept -> Coordinate<ENU> {
  Vector3d r = xyz - this->to_xyz();
  auto e = this->to_enu_matrix();
  return e * r;
}

auto Coordinate<BLH>::to_enu(const std::span<Coordinate<XYZ>>& span) const noexcept -> std::vector<Coordinate<ENU>> {
  Matrix3d e = this->to_enu_matrix();
  auto xyz0 = this->to_xyz();
  std::vector<Coordinate<ENU>> res(span.size());
  for (const auto& xyz : span) {
    Vector3d r = xyz - xyz0;
    res.emplace_back(e * r);
  }
  return res;
}

auto Coordinate<BLH>::conv_enu(const Matrix3d& conv_xyz) const noexcept -> Matrix3d {
  auto e = this->to_enu_matrix();
  return e * conv_xyz * e.transpose();
}

extern Matrix3d conv_xyz(const Coordinate<BLH>& blh, const Matrix3d& conv_enu) {
  Matrix3d res;
  auto e = blh.to_enu_matrix();
  return e.transpose() * conv_enu * e;
}

extern Matrix3d conv_xyz(const Coordinate<XYZ>& xyz, const Matrix3d& conv_enu) {
  Matrix3d res;
  auto e = xyz.to_enu_matrix();
  return e.transpose() * conv_enu * e;
}

}  // namespace navp