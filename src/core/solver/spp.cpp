#include "solution/spp.hpp"

#include <cpptrace/from_current.hpp>
#include <print>

#include "io/custom/solution_stream.hpp"

using navp::i32;
using namespace navp::solution;

class MySpp : public Spp {
 public:
  using Spp::Spp;

  virtual ~MySpp() override = default;

  void set_file(std::string_view name) {
    auto full_name = std::format("{}/{}", task_config().output().output_dir, name);
    file_ = std::make_unique<navp::io::custom::SolutionStream>(full_name, std::ios::out);
  }

 protected:
  virtual void before_action() override { rover_->logger()->info("MySpp begin"); }

  virtual void after_action() override { rover_->logger()->info("MySpp end"); }

  virtual void action() override {
    auto& ref = *rover_->station_info()->ref_pos.get();
    while (true) {
      if (next_solution()) {
        auto sol = solution();
        sol->put_record(*file_);
        navp::utils::NavVector3f64 error = sol->position.coord() - ref.coord();

        // std::println("{} : {} {}", (sol->time), error.norm(), sol->velocity.norm());
      } else {
        break;
      }
    }
  }

 private:
  std::unique_ptr<navp::io::custom::SolutionStream> file_;
};

i32 main(i32 argc, char* argv[]) {
  cpptrace::register_terminate_handler();
  navp::GlobalConfig::initialize("/root/project/nav_cxx/bin/config.toml");  // initialize config
  std::string_view config_path("/root/project/nav_cxx/config/rtk_config.toml");
  MySpp spp(config_path);
  spp.set_file("spp.sol");
  spp.run();
  return 0;
}