#include "sensors/gnss/residual.hpp"

#include <ranges>

#include "sensors/gnss/constants.hpp"

namespace navp::sensors::gnss {

ResidualBuilder::ResidualBuilder(ObsList&& _obs_list) noexcept : obs(std::move(_obs_list)) {}

ResidualBuilder::ResidualBuilder(GnssObsRecord&& _record) noexcept : obs(std::move(_record)) {}

ResidualBuilder::ResidualBuilder(std::unique_ptr<ObsList>&& _obs_ptr) noexcept : obs(std::move(_obs_ptr)) {}

ResidualBuilder::ResidualBuilder(std::unique_ptr<GnssObsRecord>&& _record_ptr) noexcept
    : obs(std::move(*_record_ptr)) {}

// todo
auto ResidualBuilder::build_spp_residual(EpochUtc t, const std::vector<const BrdcEphResult*>& eph_res_vec,
                                         const ResidualBuilderConfig& config) const noexcept
    -> utils::NavMatrixDrf64<1> {
  std::vector<Sv> sv_list;
  std::vector<f64> residual;
  for (const auto& eph_res : eph_res_vec) {
    auto sv = eph_res->sv;
    auto sv_obs = obs.query(t, sv);
    if (sv_obs.is_none()) {
      continue;
    }
    for (const auto& obs_list : sv_obs.unwrap()->sigsLists | std::views::values) {
      for (const auto& obs : obs_list) {
        sv_list.push_back(sv);
        auto row0 = (config.approximate_coord - eph_res->pos).norm();
        // correct trop
        f64 trop = 0.0;
        // correct ion
        f64 ion = 0.0;
        residual.push_back(obs.P - row0 - trop - ion + Constants::CLIGHT * eph_res->dtsv);
      }
    }
  }
  // utils::NavMatrixDrf64<1> residual_matrix(residual.data());
}
}  // namespace navp::sensors::gnss