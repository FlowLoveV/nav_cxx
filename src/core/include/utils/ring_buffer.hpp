#pragma once

#include <vector>

namespace navp::utils {

template <typename T>
class RingBuffer {
 public:
  explicit RingBuffer(size_t max_size) : max_size_(max_size), data_(max_size) {}

  void expand_to(size_t size) {
    if (size > max_size_) {
      max_size_ = size;
      data_.resize(size);
    }
  }

  void push(const T& value = T()) {
    if (data_.size() < max_size_) {
      data_.push_back(value);
    } else {
      data_[index_] = value;
      index_ = (index_ + 1) % max_size_;
    }
  }

  auto last() -> T& { return data_[index_]; }

  auto last() const -> const T& { return data_.at(index_); }

  auto data() -> std::vector<T>& { return data_; }

  auto data() const -> const std::vector<T>& { return data_; }

 private:
  std::vector<T> data_;
  size_t index_ = 0;
  size_t max_size_;
};

}  // namespace navp::utils
