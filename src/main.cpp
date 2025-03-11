#include <iostream>
#include <ranges>
#include <set>

#include "utils/eigen.hpp"

int main() {
  std::set<int> mySet = {10, 20, 30, 40, 50};

  // 使用 ranges::views::zip 和 ranges::views::iota 同时获取索引和元素
  for (auto&& [index, element] : std::views::zip(std::views::iota(0), mySet)) {
    std::cout << "Index: " << index << ", Element: " << element << std::endl;
  }

  for (auto i : std::views::iota(0, 10)) {
    std::cout << i << std::endl;
  }

  using namespace navp::utils;
  using namespace navp;

  return 0;
}