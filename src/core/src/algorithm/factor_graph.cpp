#include "algorithm/factor_graph.hpp"

namespace navp::fgo {

FactorGraph::FactorGraph() = default;

FactorGraph::~FactorGraph() = default;

GnssFactorGraph::GnssFactorGraph() = default;

GnssFactorGraph::~GnssFactorGraph() = default;

void GnssFactorGraph::set_slide_window_length(this auto& self, u8 length) noexcept { self.window_length_ = length; }
 
u8 GnssFactorGraph::slide_window_length(this const auto& self) noexcept { return self.window_length_; }

}  // namespace navp::fgo