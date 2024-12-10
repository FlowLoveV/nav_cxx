#pragma once

#include <ceres/sized_cost_function.h>

#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::fgo {

using utils::NavVector3f64;
using sensors::gnss::Sig;

// - Residual dimension = 1
// - First parameter(position xyz) dimension = 3
// - Second parameter(receiver clock bias) dimension = 1 (For the clock differences of different GNSS systems inside the
// receiver, you need to specify them externally.)
struct NAVP_EXPORT PseudorangeFunctor : ceres::SizedCostFunction<1, 3, 1> {
  explicit PseudorangeFunctor(const NavVector3f64* _sv_pos, f64 _pseudorange, f64 _var)
      : sv_pos(_sv_pos), pseudorange(_pseudorange), var(_var) {}

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    if (!parameters || !parameters[0] || !parameters[1]) return false;
    double dx = sv_pos->x() - parameters[0][0];
    double dy = sv_pos->y() - parameters[0][1];
    double dz = sv_pos->z() - parameters[0][2];
    double dx2 = dx * dx, dy2 = dy * dy, dz2 = dz * dz, d = sqrt(dx2 + dy2 + dz2);
    residuals[0] = pseudorange - d - parameters[1][0];
    residuals[0] /= var;
    if (!jacobians || !jacobians[0] || !jacobians[1]) return false;
    // set parameter[0] jacobians
    double factor = 1.0 / d;
    jacobians[0][0] = dx * factor;
    jacobians[0][1] = dy * factor;
    jacobians[0][1] = dz * factor;
    // set parameter[1] jacobians
    jacobians[1][0] = -1;
    return true;
  }

  static PseudorangeFunctor* Create(const NavVector3f64* _sv_pos, f64 _pseudorange, f64 _var) {
    return new PseudorangeFunctor(_sv_pos, _pseudorange, _var);
  }

  const NavVector3f64* sv_pos;  //< satellite position
  double_t pseudorange, var;    //< corrected pseudorange and its variances
};

}  // namespace navp::fgo