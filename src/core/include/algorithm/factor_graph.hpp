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

  void set_clock_block(ClockBlock clock_block) noexcept;
  const ClockBlock& clock_block() const noexcept;

  void set_parameter_block(ParameterBlcok parameter_block) noexcept;
  const ParameterBlcok& parameter_block() const noexcept;

  void set_slide_window_length(u8 length) noexcept;
  u8 slide_window_length() const noexcept;

 private:
  ClockBlock clock_block_;          ///> clock block
  ParameterBlcok parameter_block_;  ///> parameter block
  u8 window_length_;                ///> sliding window length
};

}  // namespace navp::fgo