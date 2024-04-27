#include "macro.hpp"

// import module time, only valid when ENMODULES is defined
#ifdef ENMODULES
NAV_IMPORT NAV_MODULE_NAME(time);
// if don't enable modules, include files
#else
#include "time.hpp"
#endif

namespace nav {}  // namespace nav
