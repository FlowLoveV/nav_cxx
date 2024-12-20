#include "spdlog/common.h"
#include "utils/logger.hpp"
#include "utils/macro.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <spdlog/spdlog-inl.h>

#include "doctest.h"

using namespace navp;

TEST_CASE("test format_num") {
  // float
  // considering the decimal point and signs, the minimum width should be 3+13+1+1 = 18
  CHECK_EQ(FORMAT_NUM("{:>18}", "{:3.13f}", 3.14), "   3.1400000000000");
  CHECK_EQ(FORMAT_NUM("{:>18}", "{:3.13f}", -3.14), "  -3.1400000000000");
  // integer
  // considering the signs,the minimum width should be 4+1 = 5
  CHECK_EQ(FORMAT_NUM("{:>5}", "{:04d}", 76), " 0076");
  CHECK_EQ(FORMAT_NUM("{:>5}", "{:04d}", -76), " -076");
}

TEST_CASE("test logger") {
  nav_log(spdlog::level::debug, "this is a debug message");
  nav_trace("this a trace message");  // can't be display
  nav_debug("{}", "another debug message");
  nav_info("this is a information message");
  nav_warn("this is a warn message");
  nav_error("this is a error message");
  nav_critical("this is a critical message");
}
