#include "macro.hpp"
#include "types.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "filter.hpp"

using namespace nav;
using namespace nav::filter;

TEST_CASE("filter") {
  Sv lhs1{ConstellationEnum::Galileo}, lhs2{ConstellationEnum::GPS};
  auto filter = std::make_shared<FilterItem<Sv>>(
      CompareOperator::Equal,
      std::initializer_list<Sv>{ConstellationEnum::GPS, ConstellationEnum::BeiDou,
                                ConstellationEnum::Glonass});
  CHECK(!filter->filter_item(lhs1, LogicOperator::Or));
  CHECK(filter->filter_item(lhs2, LogicOperator::Or));

  std::vector<std::shared_ptr<Filter>> filters;
  filters.push_back(filter);
}