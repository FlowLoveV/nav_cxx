#pragma once

#include "algorithm/parameter_block.hpp"

namespace navp::fgo {

class Factor {
 public:
  constexpr void set_parameter_block(const ParameterBlcok& parameter_block) noexcept {
    parameter_block_ = &parameter_block;
  }
  constexpr const ParameterBlcok& parameter_block() const noexcept { return *parameter_block_; }

 private:
  const ParameterBlcok* parameter_block_;
};

}  // namespace navp::fgo