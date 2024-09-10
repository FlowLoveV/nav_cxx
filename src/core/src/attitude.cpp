#include "utils/attitude.hpp"

#include <format>

#include "utils/macro.hpp"
#include "utils/angle.hpp"

namespace navp {

RotationVector::RotationVector(const Vector3d& vec) noexcept {
  auto mod = vec.norm();
  *this = {mod, vec.normalized()};
}

Eigen::Vector3d RotationVector::to_vector() const noexcept { return this->angle() * this->axis(); }

Quaternion RotationVector::to_quaternion() const noexcept { return Quaternion{*this}.normalized(); }

EulerAngle RotationVector::to_eulerAngle() const noexcept { return this->to_dcm().to_eulerAngle(); }

Dcm RotationVector::to_dcm() const noexcept { return {this->toRotationMatrix()}; }

Dcm EulerAngle::to_dcm() const noexcept {
  // auto matrix = Eigen::AngleAxisd((*this)(2), Eigen::Vector3d::UnitZ()) *
  //               Eigen::AngleAxisd((*this)(1), Eigen::Vector3d::UnitY()) *
  //               Eigen::AngleAxisd((*this)(0), Eigen::Vector3d::UnitX());
  auto [s1, c1] = navp::sin_cos(this->x());
  auto [s2, c2] = navp::sin_cos(this->y());
  auto [s3, c3] = navp::sin_cos(this->z());

  Eigen::Matrix3d C;

  C(0, 0) = c2 * c3;
  C(0, 1) = -c1 * s3 + s1 * s2 * c3;
  C(0, 2) = s1 * s3 + c1 * s2 * c3;

  C(1, 0) = c2 * s3;
  C(1, 1) = c1 * c3 + s1 * s2 * s3;
  C(1, 2) = -s1 * c3 + c1 * s2 * s3;

  C(2, 0) = -s2;
  C(2, 1) = s1 * c2;
  C(2, 2) = c1 * c2;

  return {C};
}

Quaternion EulerAngle::to_quaternion() const noexcept { return Quaternion{this->to_dcm()}.normalized(); }

RotationVector EulerAngle::to_rotationVector() const noexcept { return RotationVector{this->to_dcm()}; }

void EulerAngle::as_degress() noexcept {
    this->x() = to_degress(this->x());
    this->y() = to_degress(this->y());
    this->z() = to_degress(this->z());
}

void EulerAngle::as_radians() noexcept {
    this->x() = to_radians(this->x());
    this->y() = to_radians(this->y());
    this->z() = to_radians(this->z());
}

RotationVector Dcm::to_rotationVector() const noexcept { return RotationVector{*this}; }

EulerAngle Dcm::to_eulerAngle() const noexcept {
  auto& dcm = *this;
  double pitch = std::atan2(-dcm(2, 0), std::sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

  double roll, yaw;

  if (std::abs(dcm(2, 0)) < 0.999) {
    roll = std::atan2(dcm(2, 1), dcm(2, 2));
    yaw = std::atan2(dcm(1, 0), dcm(0, 0));
  } else {
    if (dcm(2, 0) >= 0.999) {
      yaw = navp::CST_PI + std::tan((dcm(1, 2) + dcm(0, 1)) / (dcm(0, 2) - dcm(1, 1)));
    } else {
      yaw = std::tan((dcm(1, 2) - dcm(0, 1)) / (dcm(0, 2) + dcm(1, 1)));
    }
  }

  return Vector3d(roll, pitch, yaw);
}

Quaternion Dcm::to_quaternion() const noexcept { return Quaternion{*this}.normalized(); }

RotationVector Quaternion::to_rotationVector() const noexcept { return RotationVector{*this}; }

Dcm Quaternion::to_dcm() const noexcept { return Dcm{this->toRotationMatrix()}; }

EulerAngle Quaternion::to_eulerAngle() const noexcept { return this->to_dcm().to_eulerAngle(); }

auto EulerAngle::format_as_string() const noexcept -> std::string {
  static constexpr std::string _fixed = "{:>12}";
  static constexpr std::string _fmt = "{:4.6f}";
  return std::format("{} {} {}", FORMAT_NUM(_fixed, _fmt, this->x()), FORMAT_NUM(_fixed, _fmt, this->y()),
                     FORMAT_NUM(_fixed, _fmt, this->z()));
}

}  // namespace navp