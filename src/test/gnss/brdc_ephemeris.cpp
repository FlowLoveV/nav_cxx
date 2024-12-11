#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <print>

#include "../doctest.h"
#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

using namespace navp;
using namespace navp::io::rinex;
using namespace navp::sensors::gnss;

TEST_CASE("GPS BDS BRDC") {
  std::string nav_bds_path = "../test_resources/SPP/NovatelOEM20211114-01.nav";
  std::string nav_gps_path = "../test_resources/SPP/NovatelOEM20211114-01.21N";
  std::string obs_path = "../test_resources/SPP/NovatelOEM20211114-01-GPS&BDS-Double.obs";

  RinexStream bds_nav_stream(nav_bds_path, std::ios::in);
  RinexStream gps_nav_stream(nav_gps_path, std::ios::in);
  RinexStream obs_stream(obs_path, std::ios::in);

  // decode navigation record
  GnssNavRecord bds_nav, gps_nav;
  GnssObsRecord obs;
  obs.set_storage(-1);
  bds_nav.get_record(bds_nav_stream);
  gps_nav.get_record(gps_nav_stream);

  // decode all obs record
  while (!obs_stream.eof()) {
    obs.get_record(obs_stream);
  }

  EphemerisSolver eph_solver;
  eph_solver.add_ephemeris(bds_nav.nav.get());
  eph_solver.add_ephemeris(gps_nav.nav.get());
  auto ref_epoch = EpochUtc::from_gps_time<std::chrono::gps_clock>(2184, 26700);
  auto sv_vec = obs.sv_at(ref_epoch);
  auto sv_known = eph_solver.solve_sv_status(ref_epoch, sv_vec);
  auto sv_status = eph_solver.quary_sv_status_unchecked(ref_epoch, sv_known);
  // output
  for (auto sv : sv_status) {
    std::println("{}", sv->format_as_string());
  }
}
