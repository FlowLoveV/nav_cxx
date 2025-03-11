#include <benchmark/benchmark.h>

#include <vector>

#include "rtklib.h"
#include "sensors/gnss/gnss.hpp"
#include "solution/config.hpp"

std::vector<std::string> get_rinex_obs_paths() {
  return {"/mnt/e/mashuo/project/data/nav_test/rtk/02/Rove2-Double.obs"};
}

static void rtklib_rinex_obs(benchmark::State& state) {
  std::vector<std::string> filepaths = get_rinex_obs_paths();
  obs_t obs = {0};
  nav_t nav = {0};
  sta_t sta = {};

  for (auto _ : state) {
    for (const std::string& filepath : filepaths) {
      if (readrnx(filepath.c_str(), 0, "", &obs, &nav, &sta) <= 0) {
        state.SkipWithError("Failed to read RINEX file.");
        return;
      }
    }
  }
}

BENCHMARK(rtklib_rinex_obs)->Iterations(2)->MinWarmUpTime(1);

static void nav_read_rinex(benchmark::State& state) {
  using namespace navp::sensors::gnss;
  using namespace navp::solution;
  navp::GlobalConfig::initialize("/root/project/nav_cxx/config/config.toml");
  auto handler = navp::GlobalConfig::get_station_mt("static_rover001");
  for (auto _ : state) {
    while (handler->update_record()) {
    }
  }
}

BENCHMARK(nav_read_rinex)->Iterations(20)->MinWarmUpTime(1);

BENCHMARK_MAIN();
