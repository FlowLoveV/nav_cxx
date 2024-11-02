#pragma once

#include <cmath>
#include <iostream>

#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::utils {

class RotationVector;
class EulerAngle;
class Dcm;
class Quaternion;

class NAVP_EXPORT RotationVector : public AngleAxisf64 {
 public:
  using AngleAxisf64::AngleAxis;
  RotationVector(const NavVector3f64 &vec) noexcept;

  [[nodiscard]] NavVector3f64 to_vector() const noexcept;

  [[nodiscard]] Quaternion to_quaternion() const noexcept;
  [[nodiscard]] EulerAngle to_eulerAngle() const noexcept;
  [[nodiscard]] Dcm to_dcm() const noexcept;

  NavVector3f64 operator-(const RotationVector &other) const noexcept;

  friend std::ostream &operator<<(std::ostream &os, const RotationVector &rv) noexcept {
    os << rv.to_vector() << '\n';
    return os;
  }
};

// Rotation sequence : ZYX
// x,y and z respectively represent roll pitch yaw, RPY
class NAVP_EXPORT EulerAngle : public NavVector3f64 {
 public:
  using NavVector3f64::Matrix;
  [[nodiscard]] Dcm to_dcm() const noexcept;
  [[nodiscard]] Quaternion to_quaternion() const noexcept;
  [[nodiscard]] RotationVector to_rotationVector() const noexcept;
  void as_degress() noexcept;
  void as_radians() noexcept;
  auto format_as_string() const noexcept -> std::string;
};

class NAVP_EXPORT Dcm : public NavMatrix33f64 {
 public:
  using NavMatrix33f64::Matrix;
  [[nodiscard]] RotationVector to_rotationVector() const noexcept;
  [[nodiscard]] EulerAngle to_eulerAngle() const noexcept;
  [[nodiscard]] Quaternion to_quaternion() const noexcept;

  friend std::ostream &operator<<(std::ostream &os, const Dcm &q) {
    os << static_cast<NavMatrix33f64>(q);
    return os;
  }
};

class NAVP_EXPORT Quaternion : public NavQuaternionf64 {
 public:
  using NavQuaternionf64::Quaternion;
  [[nodiscard]] RotationVector to_rotationVector() const noexcept;
  [[nodiscard]] Dcm to_dcm() const noexcept;
  [[nodiscard]] EulerAngle to_eulerAngle() const noexcept;

  friend std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
    auto xyzw = q.coeffs();
    os << xyzw[3] << " " << xyzw[0] << " " << xyzw[1] << " " << xyzw[2] << '\n';
    return os;
  }
};

union NAVP_EXPORT Attitude {
  Quaternion qbn;
  Dcm cbn;
  EulerAngle euler;
};
}  // namespace navp::utils
