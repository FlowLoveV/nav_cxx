// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2023 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// Minimize 0.5 (10 - x)^2 using jacobian matrix computed using
// numeric differentiation.
#include "ceres/ceres.h"
#include "utils/types.hpp"
// A cost functor that implements the residual r = 10 - x.
struct CostFunctor {
  bool operator()(const double* const x, double* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};
int main(int argc, char** argv) {
  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  // double x = 0.5;
  // const double initial_x = x;
  // // Build the problem.
  // ceres::Problem problem;
  // // Set up the only cost function (also known as residual). This uses
  // // numeric differentiation to obtain the derivative (jacobian).
  // ceres::CostFunction* cost_function =
  //     new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 1>(new CostFunctor);
  // auto id = problem.AddResidualBlock(cost_function, nullptr, &x);
  // problem.RemoveResidualBlock(id);
  // // Run the solver!
  // ceres::Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << "\n";
  // std::cout << "x : " << initial_x << " -> " << x << "\n";
  navp::f128 x = 10.123456789, integer;
  auto frac = std::modf(x, &integer);
  long double frac_ = static_cast<long double>(frac);
  std::cout << frac_ << '\n';
  return 0;
}