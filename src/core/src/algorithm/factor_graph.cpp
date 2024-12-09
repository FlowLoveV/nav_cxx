#include "algorithm/factor_graph.hpp"

namespace navp::fgo {

FactorGraph::FactorGraph() = default;

FactorGraph::~FactorGraph() = default;

GnssFactorGraph::GnssFactorGraph() = default;

GnssFactorGraph::~GnssFactorGraph() = default;

void GnssFactorGraph::set_clock_block(ClockBlock clock_block) noexcept { clock_block_ = clock_block; }

const ClockBlock& GnssFactorGraph::clock_block() const noexcept { return clock_block_; }

void GnssFactorGraph::set_parameter_block(ParameterBlcok parameter_block) noexcept {
  parameter_block_ = parameter_block;
}

const ParameterBlcok& GnssFactorGraph::parameter_block() const noexcept { return parameter_block_; }

void GnssFactorGraph::set_slide_window_length(u8 length) noexcept { window_length_ = length; }

u8 GnssFactorGraph::slide_window_length() const noexcept { return window_length_; }

}  // namespace navp::fgo