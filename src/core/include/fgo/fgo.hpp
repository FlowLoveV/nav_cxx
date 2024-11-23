#pragma once

#include "utils/types.hpp"

namespace navp::fgo {
// define state vector index

struct StateBlock;

struct StateBlock {
  u8 index, length;
};

}  // namespace navp::fgo