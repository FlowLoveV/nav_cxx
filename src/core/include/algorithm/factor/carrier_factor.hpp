#pragma once

#include <ceres/sized_cost_function.h>

#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::fgo {

struct NAVP_EXPORT DdCarrierFactor : ceres::SizedCostFunction<1, 3, 1, 1> {
  ~DdCarrierFactor() override = default;

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    if (!jacobians || !jacobians[0]) return false;
    residuals[0] = dd_carrier - dd_carrier0;
    residuals[0] /= std;  // todo 此处可能存在问题
    // set parameter[0] jacobians
    jacobians[0][0] = jaco00;
    jacobians[0][1] = jaco01;
    jacobians[0][2] = jaco02;
    return true;
  }

 protected:
  std::byte ref_sv_prn, mov_sv_prn;
  double_t dd_carrier, dd_carrier0, jaco00, jaco01, jaco02, std;
};

}  // namespace navp::fgo
