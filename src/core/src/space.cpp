#include "space_time/space.hpp"

#include "rtklib.h"

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

auto Coordinate<XYZ>::to_enu(const std::span<Coordinate>& span) const noexcept
    -> std::vector<Coordinate<ENU>> {
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

auto Coordinate<XYZ>::to_enu(const std::span<Coordinate<BLH>>& span) const noexcept
    -> std::vector<Coordinate<ENU>> {
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

auto Coordinate<BLH>::to_enu(const std::span<Coordinate>& span) const noexcept
    -> std::vector<Coordinate<ENU>> {
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

auto Coordinate<BLH>::to_enu(const std::span<Coordinate<XYZ>>& span) const noexcept
    -> std::vector<Coordinate<ENU>> {
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