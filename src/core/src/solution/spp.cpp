#include "solution/spp.hpp"

#include "algorithm/wls.hpp"
#include "sensors/gnss/constants.hpp"
#include "solution/config.hpp"

using namespace navp::sensors::gnss;

namespace navp::solution {

using sensors::gnss::Sv;

Spp::Spp(std::shared_ptr<GnssHandler> handler) noexcept : rover_(handler) {}

std::shared_ptr<GnssHandler> Spp::gnss_handler() noexcept { return rover_; }

Option<PvtSolutionRecord> Spp::next_solution() noexcept {
  PvtSolutionRecord sol;
  if (!rover_->update_record()) {
    return None;
  }
  auto info = rover_->update_runtime_info();
  auto settings = rover_->settings();
  auto& clock_map = settings->clock_map;
  sol.time = info->epoch;  // set time

  auto obs_handler = rover_->generate_rawobs_handler();
  AtmosphereHandlerMap atmosphere_handler_map;  //  tmosphere handler
  u16 observation_size = 0;
  std::ranges::for_each(obs_handler, [&observation_size, &atmosphere_handler_map, this](const RawObsHandler& handler) {
    Sv sv = handler.obs->sv;
    atmosphere_handler_map.insert({sv, rover_->generate_atmosphere_handler(sv)});  // atmosphere
    // random
    auto random_handler = rover_->generate_random_handler(sv).set_options(GnssRandomHandler::Pseudorange);
    for (auto sig : handler.sig) {
      random_handler.handle(sig);
    }
    observation_size += handler.sig.size();
  });

  algorithm::WeightedLeastSquare<f64> wls(3 + clock_map->size(), observation_size);

  auto sig_handler = [&sol, &wls, &clock_map](const Sig& sig, const AtmosphereHandler& atmos_handler, u16 index) {
    f64 distance = 0;
    auto sv_info = atmos_handler.sv_info();
    auto sv = sv_info->sv;
    auto clock_index = clock_map->at(sv.system());

    auto& jacobian = wls.jacobian();
    auto& observation = wls.observation();
    auto& parameter = wls.parameter();
    auto& correction = wls.parameter_correction();
    auto& weight = wls.weight();

    sv_info->view_vector_to(sol.position, jacobian(index, 0), jacobian(index, 1), jacobian(index, 2), distance);
    observation(index) = sig.pseudorange - distance - parameter(clock_index) + Constants::CLIGHT * sv_info->dtsv;
    // when the position correction < 1, correct trop and iono
    if (correction(0) != 0.0 && correction(0) < 1) {
      observation(index) -= atmos_handler.handle_trop(&sol.blh);
      observation(index) -= atmos_handler.handle_iono(&sol.blh);
    }
    // set weight (notice : here is the observation variance, not weight, need to inverse the weight later)
    weight(index, index) = sig.code_var;
  };

SppLoop: {
  u16 index = 0;
  std::ranges::for_each(obs_handler, [&](const RawObsHandler& handler) {
    // set wls
    for (auto sig : handler.sig) {
      sig_handler(*sig, atmosphere_handler_map.at(handler.obs->sv), index++);
    }
    // inverse convariance to weight
    wls.weight() = wls.weight().inverse();
    // wls correct
    wls.correct();
  });
}

  u8 iteration = 0;

  while (true) {
    goto SppLoop;
    auto& parameter_correction = wls.parameter_correction();
    // iteration end
    if (parameter_correction.block(0, 0, 3, 1).norm() < 1e-6 || iteration > 9) {
      wls.evaluate();
      break;
    }

    ++iteration;
  }

  return sol;
}

}  // namespace navp::solution