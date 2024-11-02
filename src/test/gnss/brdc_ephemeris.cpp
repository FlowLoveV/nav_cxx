#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <print>

#include "../doctest.h"
#include "io/rinex/rinex_record.hpp"
#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/broadcast_eph.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

using namespace navp::io::rinex;
using namespace navp::sensors::gnss;

TEST_CASE("GPS BDS BRDC") {
  std::string nav_bds_path = "../test_resources/SPP/NovatelOEM20211114-01.nav";
  std::string nav_gps_path = "../test_resources/SPP/NovatelOEM20211114-01.21N";
  std::string obs_path = "../test_resources/SPP/NovatelOEM20211114-01-GPS&BDS-Double.obs";

  CHECK(sizeof(navp::io::Stream) == 568);
  RinexStream bds_nav_stream(nav_bds_path, std::ios::in);
  RinexStream gps_nav_stream(nav_gps_path, std::ios::in);
  RinexStream obs_stream(obs_path, std::ios::in);

  // decode navigation record
  GnssNavRecord bds_nav, gps_nav;
  GnssObsRecord obs;
  bds_nav.get_record(bds_nav_stream);
  gps_nav.get_record(gps_nav_stream);

  // decode all obs record
  while (!obs_stream.eof()) {
    obs.get_record(obs_stream);
  }

  BrdcEphSolver bds_eph_solver(bds_nav);
  BrdcEphSolver gps_eph_solver(gps_nav);

  auto beg_time = obs.begin_time();
  navp::utils::GTime ref_gtime;
  ref_gtime.bigTime = 2184.0 * 604800.0 + 26700.0;
  auto ref_epoch = Epoch<navp::UTC>(ref_gtime);
  auto sv_vec = obs.sv(ref_epoch).unwrap();
  auto bds_sv_known = bds_eph_solver.solve_sv_stat(ref_epoch, sv_vec);
  auto gps_sv_known = gps_eph_solver.solve_sv_stat(ref_epoch, sv_vec);
  auto bds_sv_status = bds_eph_solver.quary_sv_status_unchecked(ref_epoch, bds_sv_known);
  auto gps_sv_status = gps_eph_solver.quary_sv_status_unchecked(ref_epoch, gps_sv_known);
  // output
  for (auto sv : bds_sv_status) {
    std::println("{}", sv->format_as_string());
  }
  for (auto sv : gps_sv_status) {
    std::println("{}", sv->format_as_string());
  }
}
