#pragma once

#include "utils/eigen.hpp"

namespace navp::algorithm {

template <std::floating_point _Float_t>
class WeightedLeastSquare;

template <std::floating_point _Float_t>
class WeightedLeastSquare {
 public:
  using DynamicVector = Eigen::Vector<_Float_t, Eigen::Dynamic>;
  using DynamicMatrix = Eigen::Matrix<_Float_t, Eigen::Dynamic, Eigen::Dynamic>;

  constexpr WeightedLeastSquare(size_t parameter_size, size_t observation_size)
      : parameter_(parameter_size),
        parameter_correction_(parameter_size),
        observation_(observation_size),
        observation_correction_(observation_size),
        jacobian_(observation_size, parameter_size),
        weight_(observation_size, observation_size),
        cofactor_(parameter_size, parameter_size),
        sigma_(100) {
    reset_();
  }

  ~WeightedLeastSquare() = default;

  void correct() {
    cofactor_ = jacobian_.transpose() * weight_ * jacobian_;
    parameter_correction_ = cofactor_.inverse() * jacobian_.transpose() * weight_ * observation_;
    parameter_ += parameter_correction_;
  }

  void evaluate() {
    observation_correction_ = jacobian_ * parameter_correction_ - observation_;
    auto observation_size = observation_.size(), parameter_size = parameter_.size();
    _Float_t r = observation_size - parameter_size;
    if (r > 0) {
      sigma_ = (observation_correction_.transpose() * weight_ * observation_correction_)(0, 0) / r;
    }
  }

  constexpr inline const DynamicVector& parameter() const noexcept { return parameter_; }
  constexpr inline DynamicVector& parameter() noexcept { return parameter_; }

  constexpr inline const DynamicVector& observation() const noexcept { return observation_; }
  constexpr inline DynamicVector& observation() noexcept { return observation_; }

  constexpr inline const DynamicMatrix& jacobian() const noexcept { return jacobian_; }
  constexpr inline DynamicMatrix& jacobian() noexcept { return jacobian_; }

  constexpr inline const DynamicMatrix& weight() const noexcept { return weight_; }
  constexpr inline DynamicMatrix& weight() noexcept { return weight_; }

  constexpr inline const DynamicVector& parameter_correction() const noexcept { return parameter_correction_; }
  constexpr inline const DynamicVector& observation_correction() const noexcept { return observation_correction_; }
  constexpr inline const DynamicMatrix& cofactor() const noexcept { return cofactor_; }

 protected:
  void reset_() noexcept {
    parameter_.setZero();
    parameter_correction_.setZero();
    observation_.setZero();
    observation_correction_.setZero();
    jacobian_.setZero();
    weight_.setZero();
    cofactor_.setZero();
  }
  // L = HX            Taylor first-order expansion
  // Q = H'P^{-1}H
  // dx = QH'PL
  // X = X + dx
  // v = Hdx - L
  // σ = v'P^{-1}v / (observation_size - parameter_size)
  DynamicVector parameter_;               // estimated parameter      X
  DynamicVector parameter_correction_;    // parameter correction     dx
  DynamicVector observation_;             // observation vector       L
  DynamicVector observation_correction_;  // observation correction   v
  DynamicMatrix jacobian_;                // jacobian matrix          H
  DynamicMatrix weight_;                  // weight matrix            P
  DynamicMatrix cofactor_;                // cofactor matrix          Q
  _Float_t sigma_;                        // sigma                    σ
};

}  // namespace navp::algorithm