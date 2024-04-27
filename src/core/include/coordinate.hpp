#include "macro.hpp"

// export module time, only valid when ENMODULES is defined
NAV_EXPORT NAV_MODULE NAV_MODULE_NAME(coordinate);
NAV_IMPORT STD_MODULE;
// if don't enable modules, include files
#ifndef ENMODULES
#pragma once
#include <iostream>
#endif

namespace nav {

NAV_EXPORT void test();

}  // namespace nav