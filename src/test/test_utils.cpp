#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <print>

#include "doctest.h"
#include "utils/attitude.hpp"

TEST_CASE("attitude") {
  using namespace navp::utils;
  auto euler = EulerAngle{-123.1231, 20.132132, -359.00};
  std::println("{}", euler.format_as_string());
}