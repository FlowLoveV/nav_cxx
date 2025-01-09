#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Eigen>

#include "utils/exception.hpp"
#include "utils/types.hpp"

namespace navp::algorithm {

template <std::floating_point _Float_t, bool _Throw_On_NaN>
class WeightedLeastSquare;

REGISTER_NAV_RUNTIME_ERROR_CHILD(WlsRuntimeError, NavRuntimeError);

#define THROW_OR_LOG(message)         \
  {                                   \
    if constexpr (_Throw_On_NaN) {    \
      throw WlsRuntimeError(message); \
    } else {                          \
      logger_->error(message);        \
    }                                 \
  }

template <std::floating_point _Float_t, bool _Throw_On_NaN = true>
class WeightedLeastSquare {
 public:
  using DynamicVector = Eigen::Vector<_Float_t, Eigen::Dynamic>;
  using DynamicMatrix = Eigen::Matrix<_Float_t, Eigen::Dynamic, Eigen::Dynamic>;

  constexpr WeightedLeastSquare() noexcept = default;

  WeightedLeastSquare(size_t parameter_size, size_t observation_size, std::shared_ptr<spdlog::logger> logger) noexcept
      : parameter_(parameter_size),
        parameter_correction_(parameter_size),
        observation_(observation_size),
        observation_correction_(observation_size),
        jacobian_(observation_size, parameter_size),
        weight_(observation_size, observation_size),
        cofactor_(parameter_size, parameter_size),
        sigma_(100),
        logger_(logger) {
    reset_();
  }

  constexpr WeightedLeastSquare(const WeightedLeastSquare&) = delete;
  constexpr WeightedLeastSquare& operator=(const WeightedLeastSquare&) = delete;
  constexpr WeightedLeastSquare(WeightedLeastSquare&&) = default;
  constexpr WeightedLeastSquare& operator=(WeightedLeastSquare&&) = default;

  ~WeightedLeastSquare() = default;

  void correct() {
    detect_parameter();    // detect if parameter is nan
    detect_weight();       // detect if weight is nan
    detect_jacobian();     // detect if jacobian is nan
    detect_observation();  // detect if observation is nan
    cofactor_ = jacobian_.transpose() * weight_ * jacobian_;
    detect_cofactor();  // detect if cofactor is nan
    parameter_correction_ = cofactor_.inverse() * jacobian_.transpose() * weight_ * observation_;
    // logger_->info("parameter correction:\n{}", parameter_correction_.transpose());
    detect_parameter_correction();  // detect if parameter correction is nan
    parameter_ += parameter_correction_;
  }

  void evaluate() {
    detect_jacobian();              // detect if jacobian is nan
    detect_parameter_correction();  // detect if parameter correction is nan
    detect_observation();           // detect if observation is nan
    observation_correction_ = jacobian_ * parameter_correction_ - observation_;
    detect_observation_correction();  // detect if observation correction is nan
    auto observation_size = observation_.size(), parameter_size = parameter_.size();
    _Float_t r = observation_size - parameter_size;
    if (r > 0) {
      sigma_ = (observation_correction_.transpose() * weight_ * observation_correction_)(0, 0) / r;
    }
  }

  constexpr inline size_t parameter_size() const noexcept { return parameter_.size(); }

  constexpr inline size_t observation_size() const noexcept { return observation_.size(); }

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
  constexpr inline const f64 sigma() const noexcept { return sigma_; }

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

  inline void detect_parameter() {
    if (parameter_.hasNaN()) THROW_OR_LOG("parameter has NaN");
  }

  inline void detect_observation() {
    if (observation_.hasNaN()) {
      THROW_OR_LOG("observation has NaN");
    }
  }

  inline void detect_jacobian() {
    if (jacobian_.hasNaN()) THROW_OR_LOG("jacobian has NaN");
  }

  inline void detect_weight() {
    if (weight_.hasNaN()) THROW_OR_LOG("weight has NaN");
  }

  inline void detect_cofactor() {
    if (cofactor_.hasNaN()) THROW_OR_LOG("cofactor has NaN");
  }

  inline void detect_parameter_correction() {
    if (parameter_correction_.hasNaN()) THROW_OR_LOG("parameter correction has NaN");
  }

  inline void detect_observation_correction() {
    if (observation_correction_.hasNaN()) THROW_OR_LOG("observation correction has NaN");
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

  std::shared_ptr<spdlog::logger> logger_;  // logger
};

#undef THROW_OR_LOG

}  // namespace navp::algorithm