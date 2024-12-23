#include "solution/spp.hpp"

#include <cpptrace/from_current.hpp>

using navp::i32;

i32 main(i32 argc, char* argv[]) {
  using namespace navp::solution;
  // initialize config
  navp::GlobalConfig::initialize("/root/project/nav_cxx/bin/config.toml");
  cpptrace::register_terminate_handler();

  Spp spp(navp::GlobalConfig::get_station_mt("default"));
  auto sol = spp.next_solution();
  if (sol.is_some()) {
  }
  return 0;
}