#include "algorithm/parameter_block.hpp"

namespace navp {

void ParameterBlockHandler::set_clock_block(ClockBlock clock_block) noexcept { clock_block_ = clock_block; }

const ClockBlock& ParameterBlockHandler::clock_block() const noexcept { return clock_block_; }

void ParameterBlockHandler::set_parameter_block(ParameterBlcok parameter_block) noexcept {
  parameter_block_ = parameter_block;
}

const ParameterBlcok& ParameterBlockHandler::parameter_block() const noexcept { return parameter_block_; }

}  // namespace navp