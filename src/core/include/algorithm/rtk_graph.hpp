#pragma once

#include "algorithm/factor/pseudorange_factor.hpp"
#include "algorithm/factor_graph.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/time.hpp"

namespace navp::fgo {

using sensors::gnss::Sv;

class RtkFactorGraph;

class NAVP_EXPORT RtkFactorGraph : public GnssFactorGraph {
 public:
  RtkFactorGraph();

  virtual ~RtkFactorGraph();

 protected:
  struct Residuals {
    double_t dd_pseudorange, dd_doppler;
  };

  std::map<EpochUtc, std::vector<double_t>> state_;  // parameters

  std::map<EpochUtc, std::vector<Residuals>> residual_;  // residuals

  std::map<EpochUtc, std::vector<ceres::ResidualBlockId[2]>> residual_id_;  // residuals id
};

}  // namespace navp::fgo