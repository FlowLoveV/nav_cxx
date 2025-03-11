#pragma once

#include "algorithm/wls.hpp"
#include "sensors/gnss/gnss.hpp"
#include "solution/solution.hpp"
#include "solution/task.hpp"
#include "utils/ring_buffer.hpp"

namespace navp::solution {

using sensors::gnss::ClockParameterMap;
using sensors::gnss::CodeMap;
using sensors::gnss::GnssHandler;

class Spp;

struct __SppPayload {
  typedef std::vector<sensors::gnss::GnssRawObsHandler> ObsHandlerType;
  typedef std::vector<f64> AtmosphereError;

  __SppPayload& _set_maskfilters(const TaskConfig& config) noexcept;

  __SppPayload& _set_information(std::shared_ptr<GnssHandler>& handler) noexcept;

  __SppPayload& _set_obs_handler(std::shared_ptr<GnssHandler>& handler) noexcept;

  __SppPayload& _set_clock_map(const std::shared_ptr<GnssHandler>& handler) noexcept;

  __SppPayload& _set_wls(u32 parameter_size, u32 observation_size, std::shared_ptr<spdlog::logger> logger) noexcept;

  __SppPayload& _set_atmosphere_error(u16 number) noexcept;

  __SppPayload& _set_solution(PvtSolutionRecord* sol) noexcept;

  auto _raw_obs_at(u16 index) const noexcept -> const sensors::gnss::GnssRawObsHandler&;

  auto _trop_error_at(u16 index) const noexcept -> f64;

  auto _iono_error_at(u16 index) const noexcept -> f64;

  inline auto _wls() const noexcept -> const algorithm::WeightedLeastSquare<f64>& { return *wls_; }

  inline auto _wls() noexcept -> algorithm::WeightedLeastSquare<f64>& { return *wls_; }

  inline auto _trop_error() const noexcept -> const AtmosphereError& { return *trop_error_; }

  inline auto _trop_error() noexcept -> AtmosphereError& { return *trop_error_; }

  inline auto _iono_error() const noexcept -> const AtmosphereError& { return *iono_error_; }

  inline auto _iono_error() noexcept -> AtmosphereError& { return *iono_error_; }

  void _calculate_atmosphere_error(TropModelEnum trop, IonoModelEnum iono) noexcept;

  f64 _position_iter_once() noexcept;

  f64 _velocity_iter_once() noexcept;

  void _position_evaluate() noexcept;

  void _velocity_evaluate() noexcept;

  EpochUtc epoch() const noexcept;

  u16 satellite_number() const noexcept;

  u16 signal_number() const noexcept;

  u8 clock_parameter_number() const noexcept;

  u8 clock_parameter_index(sensors::gnss::Sv sv) const noexcept;

  void _handle_variance(sensors::gnss::RandomModelEnum model) const noexcept;

  void _reset() noexcept;

  bool _position_solvable() const noexcept;

  bool _velocity_solvable() const noexcept;

 private:
  const sensors::gnss::GnssRuntimeInfo* info_;                // current epoch information
  std::unique_ptr<ObsHandlerType> obs_handler_;               // observation handler
  mutable ClockParameterMap clock_map_;                       // clock parameter map
  std::unique_ptr<AtmosphereError> iono_error_, trop_error_;  // atmosphere error
  std::unique_ptr<algorithm::WeightedLeastSquare<f64>> wls_;  // weighted least square
  const filter::MaskFilters* filters_;                        // maskfilters
  PvtSolutionRecord* sol_;                                    // solution
};

class NAVP_EXPORT Spp : protected __SppPayload {
  friend class Rtk;

 public:
  Spp(const TaskConfig& task_config, bool enabled_mt = false);

  using __SppPayload::epoch;

  auto station() const noexcept -> const GnssHandler*;

  bool load_next_epoch() noexcept;

  bool solve() noexcept;

  auto solution() const noexcept -> const PvtSolutionRecord*;

  virtual bool solve_position() noexcept;

  virtual bool solve_velocity() noexcept;

  ~Spp() = default;

 protected:
  void load_spp_payload() noexcept;
  virtual void model_spp_position() noexcept;
  virtual void model_spp_velocity() noexcept;

  utils::RingBuffer<PvtSolutionRecord> solution_;  // solution
  std::shared_ptr<GnssHandler> rover_;             // rover station
};

class NAVP_EXPORT SppServer : public Task, public Spp {
 public:
  SppServer(std::string_view cfg_path, bool enabled_mt = false);

  virtual ~SppServer() override = default;
};

}  // namespace navp::solution