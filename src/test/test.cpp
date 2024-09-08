#include "errors.hpp"
#include "logger.hpp"
#include "macro.hpp"
#include "rtklib.h"
#include "spdlog/common.h"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

TEST_CASE("test open rtklib trace") {
  traceopen("../log/rtklib/trace.txt");
  tracelevel(5);
}

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
  nav_data_export(FORMAT_NUM(":>18", ":3.13f", 3.1415));
  nav_log(spdlog::level::debug, "this is a debug message");
  nav_trace("this a trace message");  // can't be display
  nav_debug("{}", "another debug message");
  nav_info("this is a information message");
  nav_warn("this is a warn message");
  nav_error("this is a error message");
  nav_critical("this is a critical message");
}

TEST_CASE("test errors") {
  using namespace navp;
  NavError(ErrorId::DividZero).warn();
  NavError(ErrorId::DividZero).crash();
}

/*******************************************************************************************
test rtklib
********************************************************************************************/

// TEST_CASE("rtklib rinex version 3.xx ephemeris") {
//   std::string path = "../test_resources/NAV/V3/AMEL00NLD _R_20210010000_01D_MN.rnx";
//   obs_t o;
//   nav_t n;
//   sta_t s;
//   auto res = readrnx(path.c_str(), 1, "", &o, &n, &s);
//   CHECK_EQ(res, 1);
// }

// from exprtk/exprtk_simple_example_01.cpp
// TEST_CASE("exprtk using") {
//   using T = double;
//   typedef exprtk::symbol_table<T> symbol_table_t;
//   typedef exprtk::expression<T> expression_t;
//   typedef exprtk::parser<T> parser_t;
//   const std::string expression_string =
//       "clamp(-1.0,sin(2 * pi * x) + cos(x / 2 * pi),+1.0)";
//   T x;
//   symbol_table_t symbol_table;
//   symbol_table.add_variable("x", x);
//   symbol_table.add_constants();
//   expression_t expression;
//   expression.register_symbol_table(symbol_table);
//   parser_t parser;
//   parser.compile(expression_string, expression);
//   x = T{0};
//   const T y = expression.value();
//   CHECK_EQ(y, 1.0);
// }
