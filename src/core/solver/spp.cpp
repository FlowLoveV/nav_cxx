#include "solution/spp.hpp"

#include <cpptrace/from_current.hpp>
#include <print>

using navp::i32;
using namespace navp::solution;

class MySpp : public Spp {
 public:
  using Spp::Spp;

  virtual ~MySpp() override = default;

 protected:
  virtual void before_action() override { rover_->logger()->info("MySpp begin"); }

  virtual void after_action() override { rover_->logger()->info("MySpp end"); }

  virtual void action() override {
    auto& ref = *rover_->station_info()->ref_pos.get();
    while (true) {
      if (next_solution()) {
        auto sol = solution();
        navp::utils::NavVector3f64 error = sol->position.coord() - ref.coord();
        std::println("{} : {} {}", (sol->time), error.norm(), sol->velicity.norm());
      } else {
        break;
      }
    }
  }
};

i32 main(i32 argc, char* argv[]) {
  cpptrace::register_terminate_handler();
  navp::GlobalConfig::initialize("/root/project/nav_cxx/bin/config.toml");  // initialize config
  std::string_view config_path("/root/project/nav_cxx/config/rtk_config.toml");
  MySpp spp(config_path);
  spp.run();
  return 0;
}