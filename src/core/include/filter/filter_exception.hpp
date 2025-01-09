#pragma once

#include "utils/exception.hpp"

namespace navp::filter {

REGISTER_NAV_RUNTIME_ERROR_CHILD(NavFilterRuntimeError, NavRuntimeError);

/*
 * Register filter runtime error
 */
REGISTER_NAV_RUNTIME_ERROR_CHILD(FilterParseOperatorError, NavFilterRuntimeError);

REGISTER_NAV_RUNTIME_ERROR_CHILD(FilterParseItemError, NavFilterRuntimeError);

REGISTER_NAV_RUNTIME_ERROR_CHILD(FilterParseError, NavFilterRuntimeError);
}  // namespace navp::filter