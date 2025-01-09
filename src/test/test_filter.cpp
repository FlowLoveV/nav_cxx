#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <cpptrace/from_current.hpp>

#include "doctest.h"
#include "filter/filter.hpp"

using namespace navp::filter;

auto item0 = FilterItem::from_str("2024-10-01 08:00:01").unwrap_throw();
auto item1 = FilterItem::from_str("G01").unwrap_throw();
auto item2 = FilterItem::from_str("15.2e").unwrap_throw();
auto item3 = FilterItem::from_str("33.7s").unwrap_throw();
auto item4 = FilterItem::from_str("BDS").unwrap_throw();
auto item5 = FilterItem::from_str("L2").unwrap_throw();

TEST_CASE("SingleFilter") {
  cpptrace::register_terminate_handler();

  auto s0 = ">=2024-10-01 08:00:00";
  auto s1 = "!=G01";
  auto s2 = ">15e";
  auto s3 = ">35s";
  auto s4 = "!=GPS";
  auto s5 = "!=L1";

  auto f0 = Filter::from_str(s0).unwrap_throw();
  auto f1 = Filter::from_str(s1).unwrap_throw();
  auto f2 = Filter::from_str(s2).unwrap_throw();
  auto f3 = Filter::from_str(s3).unwrap_throw();
  auto f4 = Filter::from_str(s4).unwrap_throw();
  auto f5 = Filter::from_str(s5).unwrap_throw();

  CHECK(f0.apply(item0) == true);
  CHECK(f1.apply(item1) == false);
  CHECK(f2.apply(item2) == true);
  CHECK(f3.apply(item3) == false);
  CHECK(f4.apply(item4) == true);
  CHECK(f5.apply(item5) == true);

  CHECK(f0.apply(item1) == true);
}

TEST_CASE("MutiFilter") {
  auto s = ">=2024-10-01 08:00:00, !=G01, >15e, >35s, !=GPS, !=L1";
  auto f = MaskFilters::from_str(s).unwrap_throw();

  CHECK(f.apply(item0) == true);
  CHECK(f.apply(item1) == false);
  CHECK(f.apply(item2) == true);
  CHECK(f.apply(item3) == false);
  CHECK(f.apply(item4) == true);
  CHECK(f.apply(item5) == true);
}