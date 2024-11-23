#include "solution/spp.hpp"

#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/broadcast_eph.hpp"
#include "solution/config.hpp"
#include "utils/error.hpp"

using namespace navp::io::rinex;
using navp::errors::NavError;

namespace navp::solution {

Spp::Spp(std::string_view cfg_path) noexcept : ConfigTask(cfg_path), eph_solver_(std::make_unique<BrdcEphSolver>()) {
  init();
}

Spp::~Spp() = default;

void Spp::solve() {
  while (!obs_stream_->eof()) {
    PvtSolution sol;
    obs_.get_record(*obs_stream_);
    t_ = obs_.end_time();
    sol.time = t_;  // set solution time

    auto obs_map = obs_.query(t_);
    // no gnss observation
    if (!obs_map) {
      nav_debug("no gnss observation data at {}", t_);
      sol_.emplace_back(sol);
      continue;
    }
    // todo filter
    
    // solve ephemeris
    eph_solver_->solve_sv_status(t_, *obs_map);

    // solve position
    sol.mode = SolutionMode::SINGLE;
    sol_.emplace_back(sol);
  }
}

void Spp::init() {
  // init obs stream
  auto obs_path = config_[IoCfg][ObsPath].as_string()->get();
  auto rinexStream = std::make_unique<RinexStream>(obs_path);
  obs_stream_ = std::move(rinexStream);
  // read navigation
  auto nav_path_vec = config_[IoCfg][NavPath].as_array();
  if (nav_path_vec) {
    nav_.reserve(nav_path_vec->size());
    nav_path_vec->for_each([&](const toml::node& node) {
      if (const auto value = node.value<std::string>(); value.has_value()) {
        RinexStream nav_stream(value.value(), std::ios::in);
        GnssNavRecord nav_record;
        nav_record.get_record(nav_stream);
        nav_.emplace_back(std::move(nav_record));
      } else {
        nav_error("parsing \'nav_path\' field error, should be a string array!");
        exit(NavError::Initalize::Configuration::ParseConfigurationError);
      }
    });
  } else {
    nav_error("parsing \'nav_path\' field error, should be a string array!");
    exit(NavError::Initalize::Configuration::ParseConfigurationError);
  }
  // init ephemeris solver
  std::ranges::for_each(nav_, [&](const GnssNavRecord& record) { eph_solver_->add_ephemeris(record.nav.get()); });
}

}  // namespace navp::solution