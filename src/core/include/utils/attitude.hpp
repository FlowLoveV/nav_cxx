#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

namespace navp {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class RotationVector;
class EulerAngle;
class Dcm;
class Quaternion;

class RotationVector : public Eigen::AngleAxisd {
 public:
  using Eigen::AngleAxis<double>::AngleAxis;
  RotationVector(const Vector3d &vec) noexcept;

  [[nodiscard]] Eigen::Vector3d to_vector() const noexcept;

  [[nodiscard]] Quaternion to_quaternion() const noexcept;
  [[nodiscard]] EulerAngle to_eulerAngle() const noexcept;
  [[nodiscard]] Dcm to_dcm() const noexcept;

  Eigen::Vector3d operator-(const RotationVector &other) const noexcept;

  friend std::ostream &operator<<(std::ostream &os, const RotationVector &rv) noexcept {
    os << rv.to_vector() << '\n';
    return os;
  }
};

// Rotation sequence : ZYX
// x,y and z respectively represent roll pitch yaw, RPY
class EulerAngle : public Eigen::Vector3d {
 public:
  using Eigen::Vector3<double>::Vector3;
  [[nodiscard]] Dcm to_dcm() const noexcept;
  [[nodiscard]] Quaternion to_quaternion() const noexcept;
  [[nodiscard]] RotationVector to_rotationVector() const noexcept;
  void as_degress() noexcept;
  void as_radians() noexcept;
  auto format_as_string() const noexcept -> std::string;
};

class Dcm : public Eigen::Matrix3d {
 public:
  using Eigen::Matrix3<double>::Matrix3;
  [[nodiscard]] RotationVector to_rotationVector() const noexcept;
  [[nodiscard]] EulerAngle to_eulerAngle() const noexcept;
  [[nodiscard]] Quaternion to_quaternion() const noexcept;

  friend std::ostream &operator<<(std::ostream &os, const Dcm &q) {
    os << static_cast<Matrix3d>(q);
    return os;
  }
};

class Quaternion : public Eigen::Quaterniond {
 public:
  using Eigen::Quaternion<double>::Quaternion;
  [[nodiscard]] RotationVector to_rotationVector() const noexcept;
  [[nodiscard]] Dcm to_dcm() const noexcept;
  [[nodiscard]] EulerAngle to_eulerAngle() const noexcept;

  friend std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
    auto xyzw = q.coeffs();
    os << xyzw[3] << " " << xyzw[0] << " " << xyzw[1] << " " << xyzw[2] << '\n';
    return os;
  }
};

typedef union {
  Quaternion qbn;
  Dcm cbn;
  EulerAngle euler;
} Attitude;
}  // namespace navp
