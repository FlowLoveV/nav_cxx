#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <print>

#include "../doctest.h"
#include "io/rinex/rinex.hpp"
#include "sensors/gnss/broadcast_eph.hpp"

using namespace navp::io::rinex;
using namespace navp::sensors::gnss;

TEST_CASE("GPS BDS BRDC") {
  std::string nav_bds_path = "../test_resources/SPP/NovatelOEM20211114-01.nav";
  std::string nav_gps_path = "../test_resources/SPP/NovatelOEM20211114-01.21N";
  std::string obs_path = "../test_resources/SPP/NovatelOEM20211114-01-GPS&BDS-Double.obs";

  RecordGnssNav bds = RinexNav(nav_bds_path.c_str());
  RecordGnssNav gps = RinexNav(nav_gps_path.c_str());
  RecordGnssObs obs = RinexObs(obs_path.c_str());

  BrdcEphSolver bds_eph_solver(bds);
  BrdcEphSolver gps_eph_solver(gps);

  auto beg_time = obs.begin_time();
  navp::utils::GTime ref_gtime;
  ref_gtime.bigTime = 2184.0 * 604800.0 + 26700.0;
  auto ref_epoch = Epoch<navp::UTC>(ref_gtime);
  auto sv_vec = obs.sv(ref_epoch).unwrap();
  auto bds_sv_known = bds_eph_solver.solve_sv_stat(sv_vec, ref_epoch);
  auto gps_sv_known = gps_eph_solver.solve_sv_stat(sv_vec, ref_epoch);
  auto bds_sv_status = bds_eph_solver.quary_sv_status_unchecked(bds_sv_known, ref_epoch);
  auto gps_sv_status = gps_eph_solver.quary_sv_status_unchecked(gps_sv_known, ref_epoch);
  // output
  for (auto sv : bds_sv_status) {
    std::println("{}", sv->format_as_string());
  }
  for (auto sv : gps_sv_status) {
    std::println("{}", sv->format_as_string());
  }
}
