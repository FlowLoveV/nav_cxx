#include "algorithm/rtk_graph.hpp"

namespace navp::fgo {

SppFactorGraph::SppFactorGraph() = default;

SppFactorGraph::~SppFactorGraph() = default;

void SppFactorGraph::assign_latest_state(Parameters& state) noexcept {
  state = state_.size() ? state_.rbegin()->second : Parameters(clock_block().count());
}

void SppFactorGraph::init_state_at(EpochUtc time) noexcept { assign_latest_state(state_[time]); }

void SppFactorGraph::init_residual_at(EpochUtc time, Sv sv) noexcept { residual_[time][sv] = {0, 0}; }

}  // namespace navp::fgo 