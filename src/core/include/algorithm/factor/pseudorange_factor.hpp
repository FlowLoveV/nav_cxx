#pragma once

#include <ceres/sized_cost_function.h>

#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::fgo {

using utils::NavVector3f64;

// todo
//   Psedorange factor
//   Discription      Dimension         Meaning
// - Residual            1          Pseudorange residual
// - Parameter1          3          ECEF coordinate parameter(XYZ,m)
// - Parameter2          1          receiver clock bias(m)
struct NAVP_EXPORT PseudorangeFactor : ceres::SizedCostFunction<1, 3, 1> {
  ~PseudorangeFactor() override = default;

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    if (!parameters || !parameters[0] || !parameters[1]) return false;
    double dx = parameters[0][0] - sv_pos->x();
    double dy = parameters[0][1] - sv_pos->y();
    double dz = parameters[0][2] - sv_pos->z();
    double dx2 = dx * dx, dy2 = dy * dy, dz2 = dz * dz, d = sqrt(dx2 + dy2 + dz2);
    residuals[0] = pseudorange - d - parameters[1][0];
    residuals[0] /= std;
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

  static PseudorangeFactor* Create(const NavVector3f64* _sv_pos, f64 _pseudorange, f64 _var) {
    return new PseudorangeFactor(_sv_pos, _pseudorange, _var);
  }

  const NavVector3f64* sv_pos;  //< satellite position
  double_t pseudorange, std;    //< corrected pseudorange and its variances

 protected:
  explicit PseudorangeFactor(const NavVector3f64* _sv_pos, f64 _pseudorange, f64 _var)
      : sv_pos(_sv_pos), pseudorange(_pseudorange), std(_var) {}
};

//   Double differenced pseudorange factor
//   Discription      Dimension         Meaning
//  -Residual            1          Pseudorange residual
//  -Parameter1          3          ECEF coordinate correction(ΔX,ΔY,ΔZ)
struct NAVP_EXPORT DdPseudorangeFunctor : ceres::SizedCostFunction<1, 3, 1, 1> {
  virtual ~DdPseudorangeFunctor() = default;

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    if (!jacobians || !jacobians[0]) return false;
    residuals[0] = dd_pseudorange - dd_pseudorange0;
    residuals[0] /= std;  // todo 此处可能存在问题
    // set parameter[0] jacobians
    jacobians[0][0] = jaco00;
    jacobians[0][1] = jaco01;
    jacobians[0][1] = jaco02;
    return true;
  }

 protected:
  double_t dd_pseudorange, dd_pseudorange0, jaco00, jaco01, jaco02, std;
};

}  // namespace navp::fgo