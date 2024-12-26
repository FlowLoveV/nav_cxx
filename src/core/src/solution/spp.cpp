#include "solution/spp.hpp"

#include <ranges>

#include "algorithm/wls.hpp"
#include "sensors/gnss/constants.hpp"
#include "solution/config.hpp"

using namespace navp::sensors::gnss;

namespace navp::solution {

using sensors::gnss::Sv;

Spp::Spp(std::shared_ptr<GnssHandler> handler) noexcept : sol_(std::make_unique<PvtSolutionRecord>()), rover_(handler) {
  update_clock_map(rover_->record()->obs->code_map());  // init clock map
}

void Spp::update_clock_map(const CodeMap& code_map) const noexcept {
  u8 index = 0;
  std::ranges::for_each(code_map | std::views::keys, [&](ConstellationEnum cons) { clock_map_[cons] = index++; });
}

std::shared_ptr<GnssHandler> Spp::gnss_handler() noexcept { return rover_; }

auto Spp::solution() const noexcept -> const PvtSolutionRecord* { return sol_.get(); }

bool Spp::next_solution() noexcept {
  if (!rover_->update_record()) {
    return false;
  }
  auto info = rover_->update_runtime_info();
  sol_->time = info->epoch;  // set time

  auto obs_handler = rover_->generate_rawobs_handler();
  std::vector<f64> trop_corr(obs_handler.size(), 0), iono_corr(obs_handler.size(), 0);
  u16 observation_size = 0;
  std::ranges::for_each(obs_handler, [&observation_size, this](const GnssRawObsHandler& handler) {
    Sv sv = handler.obs->sv;
    auto random_handler = rover_->generate_random_handler(sv).set_options(GnssRandomHandler::Pseudorange);  // random
    for (auto sig : handler.sig) {
      random_handler.handle(sig);
    }
    observation_size += handler.sig.size();
  });

  algorithm::WeightedLeastSquare<f64> wls(3 + clock_map_.size(), observation_size);
  wls.parameter().block(0, 0, 3, 1) = sol_->position;  // init position

  auto sig_handler = [&wls, this](const Sig* sig, const EphemerisResult* sv_info, f64 trop, f64 iono, u16 index) {
    f64 distance = 0;
    auto sv = sv_info->sv;
    auto clock_index = clock_map_.at(sv.system());
    auto& jacobian = wls.jacobian();
    auto& observation = wls.observation();
    auto& parameter = wls.parameter();
    auto& correction = wls.parameter_correction();
    auto& weight = wls.weight();
    auto position = utils::CoordinateXyz(wls.parameter().block(0, 0, 3, 1));
    sv_info->view_vector_to(position, jacobian(index, 0), jacobian(index, 1), jacobian(index, 2), distance);
    observation(index) =
        sig->pseudorange - distance - Constants::CLIGHT * (parameter(clock_index) - sv_info->dtsv) - iono - trop;
    jacobian(index, 3 + clock_index) = 1;
    weight(index, index) = sig->code_var;  // set weight (notice : here is the observation variance, not weight, need to
                                           // inverse the weight later
  };

  u8 iteration = 0;  // iteration number
  while (true) {
    u16 index = 0;  // observation index
    for (u16 sat_index = 0; sat_index < obs_handler.size(); ++sat_index) {
      for (auto sig : obs_handler[sat_index].sig) {
        sig_handler(sig, obs_handler[sat_index].sv_info, trop_corr[sat_index], iono_corr[sat_index], index++);
      }
    }
    wls.weight() = wls.weight().inverse();  // inverse convariance to weight, because the previous setting was the
                                            // variance matrix, not the weight matrix

    // wls correct
    wls.correct();
    sol_->position = wls.parameter().block(0, 0, 3, 1);
    sol_->blh = sol_->position.to_blh();
    auto& parameter_correction = wls.parameter_correction();
    // when the position correction < 1, do the following
    if (parameter_correction(0) != 0.0 && abs(parameter_correction(0)) < 1) {
      for (u16 sat_index = 0; sat_index < obs_handler.size(); ++sat_index) {
        auto& handler = obs_handler[sat_index];
        handler.sv_info->update_ea_from(sol_->position);  // update satellite elevation and azimuth
        trop_corr[sat_index] = handler.trop_corr(&sol_->blh, rover_->settings()->trop);
        iono_corr[sat_index] = handler.iono_corr(&sol_->blh, rover_->settings()->iono);
      }
    }

    // iteration end
    if (parameter_correction.block(0, 0, 3, 1).norm() < 1e-6 || iteration > 9) {
      wls.evaluate();
      // update solution

      break;
    }
    ++iteration;
  }

  return true;
}

}  // namespace navp::solution