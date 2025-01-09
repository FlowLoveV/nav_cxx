#include "solution/spp.hpp"

#include <cpptrace/from_current.hpp>
#include <print>

using navp::i32;

i32 main(i32 argc, char* argv[]) {
  using namespace navp::solution;
  cpptrace::register_terminate_handler();

  // initialize config
  navp::GlobalConfig::initialize("/root/project/nav_cxx/bin/config.toml");

  Spp spp(navp::GlobalConfig::get_station_st("default"));
  auto& ref = *spp.gnss_handler()->station_info()->ref_pos.get();
  spp.gnss_handler()->logger()->info("begin");
  while (true) {
    if (spp.next_solution()) {
      auto sol = spp.solution();
      navp::utils::NavVector3f64 error = sol->position.coord() - ref.coord();
      std::println("{} : {} {}", (sol->time), error.norm(), sol->velicity.norm());
    } else {
      break;
    }
  }
  spp.gnss_handler()->logger()->info("end");
  return 0;
}