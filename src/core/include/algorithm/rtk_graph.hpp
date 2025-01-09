#pragma once

#include "algorithm/factor/pseudorange_factor.hpp"
#include "algorithm/factor_graph.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/time.hpp"

namespace navp::fgo {

using sensors::gnss::Sv;

class SppFactorGraph;
class RtkFactorGraph;

class NAVP_EXPORT SppFactorGraph : public GnssFactorGraph {
  typedef ceres::ResidualBlockId FactorId;

 public:
  struct Parameters {
    Parameters() = default;
    Parameters(u8 system_num) : clock(new double[system_num]()) {}

    double position[3]{};
    double velocity[3]{};
    double* clock = nullptr;

    ~Parameters() { delete[] clock; }
  };

  struct Resuduals {
    double pseudorange, doppler;
  };

  SppFactorGraph();

  void reset();

  virtual ~SppFactorGraph() override;

 protected:
  void remove_factors_at(EpochUtc time) noexcept;

  void init_state_at(EpochUtc time) noexcept;

  void init_residual_at(EpochUtc time, Sv sv) noexcept;

  void assign_latest_state(Parameters& state) noexcept;

  std::map<EpochUtc, Parameters> state_;  // parameters

  std::map<EpochUtc, std::map<Sv, Resuduals>> residual_;  // residuals

  std::map<EpochUtc, std::map<Sv, FactorId[2]>> residual_id_;  // residuals id
};

class NAVP_EXPORT RtkFactorGraph : public GnssFactorGraph {
 public:
  RtkFactorGraph();

  void add_pseudorange_factor() noexcept;

  virtual ~RtkFactorGraph();
};

}  // namespace navp::fgo