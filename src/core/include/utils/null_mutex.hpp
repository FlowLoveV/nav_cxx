#pragma once

namespace navp::utils {

struct null_mutex {
  void lock() const noexcept {}
  void unlock() const noexcept {}
};

}  // namespace navp::utils