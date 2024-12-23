#pragma once

#include "utils/exception.hpp"

namespace navp::sensors::gnss {

REGISTER_NAV_EXCEPTION_CHILD(GnssException);

REGISTER_NAV_RUNTIME_ERROR_CHILD(GnssRuntimeError, NavRuntimeError);

/* Introduction
 * All Gnss Exception should be registered here
 * The naming of exceptions should strictly follow certain naming rules
 */

/*
 * Register gnss exception
 * Register an exception with a fixed message
 */

/*
 * Register gnss runtime error
 * Register an exception with a runtime-related message
 */
// parse string to Constellation exception
REGISTER_NAV_RUNTIME_ERROR_CHILD(GnssParseConstellationError, GnssRuntimeError);
// parse string to Sv exception
REGISTER_NAV_RUNTIME_ERROR_CHILD(GnssParseSvError, GnssRuntimeError);
// parse string to Carrier exception
REGISTER_NAV_RUNTIME_ERROR_CHILD(GnssParseCarrierError, GnssRuntimeError);

// GnssObsRecord exception
REGISTER_NAV_RUNTIME_ERROR_CHILD(GnssObsRecordError, GnssRuntimeError);

}  // namespace navp::sensors::gnss