#include "algorithm/factor_graph.hpp"

namespace navp::fgo {

FactorGraph::FactorGraph() = default;

FactorGraph::~FactorGraph() = default;

GnssFactorGraph::GnssFactorGraph() = default;

GnssFactorGraph::~GnssFactorGraph() = default;

void GnssFactorGraph::set_slide_window_length(u8 length) noexcept { window_length_ = length; }

u8 GnssFactorGraph::slide_window_length() const noexcept { return window_length_; }

}  // namespace navp::fgo