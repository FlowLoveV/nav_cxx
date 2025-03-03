#pragma once

#include <ceres/solver.h>

#include "algorithm/parameter_block.hpp"

namespace navp::fgo {

class FactorGraph;
class GnssFactorGraph;

class FactorGraph {
 public:
  FactorGraph();

  virtual ~FactorGraph();

 protected:
  ceres::Problem problem_;
  ceres::Solver solver_;
};

class GnssFactorGraph : public FactorGraph {
 public:
  GnssFactorGraph();
  virtual ~GnssFactorGraph() override;

  void set_slide_window_length(this auto& self, u8 length) noexcept;
  u8 slide_window_length(this const auto& self) noexcept;

 private:
  u8 window_length_;  ///> sliding window length
};

}  // namespace navp::fgo