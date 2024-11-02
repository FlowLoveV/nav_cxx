#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <print>

#include "../doctest.h"
#include "io/rinex/rinex_record.hpp"
#include "plot.hpp"
#include "sensors/gnss/observation.hpp"

using namespace navp::sensors::gnss;
using namespace navp::io::rinex;
using namespace navp::plotter;

// std::string obs_path = "../test_resources/SPP/NovatelOEM20211114-01-GPS&BDS-Double.obs";
// GnssObsRecord obs = RinexObs(obs_path.c_str());

// TEST_CASE("test_raw") { example_01(); }