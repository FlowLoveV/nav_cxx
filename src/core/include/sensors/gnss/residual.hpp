#pragma once

#include "sensors/gnss/broadcast_eph.hpp"
#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/observation.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

struct NAVP_EXPORT ResidualBuilderConfig;
struct NAVP_EXPORT Residual;
class NAVP_EXPORT ResidualBuilder;

struct ResidualBuilderConfig {
  /// trop model
  TropModelEnum trop_model;
  /// iono model
  IonoModelEnum iono_model;
  /// variance model
  NoiseModelEnum noise_model;
  /// approximate station coordinates
  utils::NavVector3f64 approximate_coord;
};

struct Residual {
  utils::NavMatrixDrf64<1> residual;
};

class ResidualBuilder {
 public:
  ResidualBuilder(ObsList&& _obs_list) noexcept;
  ResidualBuilder(GnssObsRecord&& _record) noexcept;
  ResidualBuilder(std::unique_ptr<ObsList>&& _obs_ptr) noexcept;
  ResidualBuilder(std::unique_ptr<GnssObsRecord>&& _record_ptr) noexcept;

  auto build_spp_residual(EpochUtc t, const std::vector<const BrdcEphResult*>& eph_res_vec,
                          const ResidualBuilderConfig& config) const noexcept -> utils::NavMatrixDrf64<1>;

 protected:
  GnssObsRecord obs;
};

}  // namespace navp::sensors::gnss