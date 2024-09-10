#include <print>

#include "cpptrace/cpptrace.hpp"
#include "rfl.hpp"
#include "rfl/json.hpp"
#include "rfl/toml.hpp"
#include "solution/spp.hpp"

void test_trace() { 
    const auto raw_trace = cpptrace::generate_raw_trace();
    cpptrace::generate_trace().print(); }

int main(int argc, char *argv[]) {
  using namespace navp;

  auto config = solution::SppConfig{.nav_path =
                                        std::vector<std::string>{
                                            "/root/project/nav_cxx/test_resources/SPP/NovatelOEM20211114-01.nav",
                                            "/root/project/nav_cxx/test_resources/SPP/NovatelOEM20211114-01.21N",
                                        },
                                    .obs_path = "/root/project/nav_cxx/test_resources/SPP/NovatelOEM20211114-01.obs",
                                    .out_path = "/root/project/nav_cxx/test_resources/SPP/res.txt",
                                    .min_e = 10,
                                    .min_s = 30,
                                    .filter = {std::vector<std::string>{
                                        "=GPS",
                                        ">=2023-10-11 20:08:03",
                                    }}};
  rfl::toml::save("/root/project/nav_cxx/config/spp_config.toml", config);
  rfl::json::save("/root/project/nav_cxx/config/spp_config.json", config);
  test_trace();
  return 0;
}