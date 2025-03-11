#pragma once

#include <set>

#include "solution/spp.hpp"
#include "solution/task.hpp"

namespace navp::solution {

using sensors::gnss::Constellation;
using sensors::gnss::Sv;

class Rtk;

struct __RtkPayload;

struct __RtkPayload {
  struct Indicator {
    i64 base_fixed : 1;
  } indicator_;

  struct SystemPayload {
    struct ObservationCache {
      f64 pseudorange, carrier;
    };
    typedef std::vector<sensors::gnss::EphemerisResult::ViewVector> ViewVectorCache;
    typedef std::vector<ObservationCache> ObservationCacheVector;

    std::vector<Sv> public_view_satellites;        // the first should be reference satellite
    std::vector<f64> bt_base_satellites_distance;  // the distance between base station and public view satellites
    std::vector<const sensors::gnss::EphemerisResult*> rover_eph;  // rover ephemeris result
    std::vector<const sensors::gnss::Sig*> rover_sigs, base_sigs;  // rover/base signals, the size should be
                                                                   // num_sigs = num_code * num_satellites;
    std::set<sensors::gnss::ObsCodeEnum> available_code_set;       // available codes
    const PvtSolutionRecord* rover_spp_sol_;                       // rover spp solution record

    mutable std::unique_ptr<ViewVectorCache> view_vector_cache;  // view vector cache
    mutable std::unique_ptr<ObservationCacheVector> btsta_sd_obs_cache, rover_btsat_sd_obs_cache,
        base_btsat_sd_obs_cache, dd_obs_cache;  // differential observation cache
    mutable std::unique_ptr<ObservationCacheVector> btsta_sd_random_cache, rover_btsat_sd_random_cache,
        base_btsat_sd_random_cache;
    mutable std::unique_ptr<utils::NavMatrixDf64> dd_weight_cache;  // differential random cache

    void select_available_sigs(const GnssHandler* rover, const GnssHandler* base,
                               const filter::MaskFilters* mask_filter) noexcept;

    inline u16 bt_station_ambiguity_size() const noexcept {
      return static_cast<u16>(available_code_set.size() * public_view_satellites.size());
    }

    inline u16 bt_satellite_ambiguity_size() const noexcept {
      return static_cast<u16>(available_code_set.size() * (public_view_satellites.size() - 1));
    }

    inline u8 dd_ambiguity_size() const noexcept {
      return static_cast<u8>(available_code_set.size() * (public_view_satellites.size() - 1));
    }

    bool is_view_vector_cached() const noexcept;
    void update_view_vector_cache() const noexcept;
    void reset_view_vector_cache() const noexcept;

    bool is_bt_sta_sd_obs_cached() const noexcept;
    void update_bt_sta_sd_obs_cache() const noexcept;
    void reset_bt_sta_sd_obs_cache() const noexcept;

    bool is_bt_sat_sd_obs_cached() const noexcept;
    void update_bt_sat_sd_obs_cache() const noexcept;
    void reset_bt_sat_sd_obs_cache() const noexcept;

    bool is_dd_obs_cached() const noexcept;
    void update_dd_obs_cache() const noexcept;
    void reset_dd_obs_cache() const noexcept;

    bool is_bt_sta_sd_random_cached() const noexcept;
    void update_bt_sta_sd_random_cache() const noexcept;
    void reset_bt_sta_sd_random_cache() const noexcept;

    bool is_bt_sat_sd_random_cached() const noexcept;
    void update_bt_sat_sd_random_cache() const noexcept;
    void reset_bt_sat_sd_random_cache() const noexcept;

    bool is_dd_weight_cached() const noexcept;
    void update_dd_weight_cache() const noexcept;
    void reset_dd_weight_cache() const noexcept;

    void handle_variance(sensors::gnss::RandomModelEnum rover_model,
                         sensors::gnss::RandomModelEnum base_model) const noexcept;

    void build_dd_jacobian(Eigen::Block<utils::NavMatrixDf64> jacobian) const noexcept;

    void build_dd_observation(Eigen::Block<utils::NavVectorDf64> observation) const noexcept;

    void build_dd_weight(Eigen::Block<utils::NavMatrixDf64> weight) const noexcept;

    void build_dd_model(algorithm::WeightedLeastSquare<f64>* wls, u16 dd_ambiguity_index) const noexcept;
  };

  __RtkPayload() = default;

  ~__RtkPayload() = default;

  auto epoch() const noexcept -> EpochUtc;

 protected:
  bool _reset(const Spp* rover, const Spp* base) noexcept;

  __RtkPayload& _set_maskfilters(const TaskConfig& config) noexcept;

  __RtkPayload& _set_wls(size_t parameter_size, size_t observation_size,
                         std::shared_ptr<spdlog::logger> logger) noexcept;

  void _build_dd_model() noexcept;

  f64 _iter_once() noexcept;

  void _update_rover_position(const utils::CoordinateXyz* pos) noexcept;

  void _fix_ambiguity(f32 ratio_threshold) noexcept;

  u16 _bt_station_ambiguity_size() const noexcept;

  u16 _bt_satellite_ambiguity_size() const noexcept;

  u8 _dd_ambiguity_size() const noexcept;

  std::unique_ptr<algorithm::WeightedLeastSquare<f64>> wls_;  // weighted least square
  std::unordered_map<ConstellationEnum, SystemPayload> system_payload_map_;
  const utils::CoordinateXyz* rover_pos_;

 private:
  bool _solvable() const noexcept;

  // filter time/constellation/sv/elevation
  // get public view satellits between base station and rover
  bool _get_public_view_satellites() noexcept;

  // place the reference satellite at first and sort the rest
  void _select_reference_satellite() noexcept;

  // get the mobile station observation value, the order is consistent with the satellite vector
  void _get_rover_information() noexcept;

  // get base station : the distance between base station and public view satellites
  void _get_base_information() noexcept;

  // filter snr/carrier
  // find available and valid signals that can be used for rtk
  void _select_available_sigs() noexcept;

  void _handle_variance() noexcept;

  const GnssHandler *rover_, *base_;        // gnsshandler pointer
  const filter::MaskFilters* mask_filter_;  // filters
  const utils::CoordinateXyz* base_pos_;    // position
};

class NAVP_EXPORT Rtk : protected __RtkPayload {
 public:
  enum RtkModel : u8 {
    DdBasic = 0,
  };

  Rtk(const TaskConfig& task_config, bool enabled_mt = false);

  ~Rtk() noexcept = default;

  bool load_next_epoch() noexcept;

  bool load_rtk_payload() noexcept;

  bool solve(RtkModel model = DdBasic) noexcept;

 protected:
  void model(RtkModel model = DdBasic) noexcept;

  void model_dd_basic() noexcept;

  void update_position_after_iter() noexcept;

  void evaluate() noexcept;

  bool align_time() noexcept;

  f32 ratio_threshold_;

  std::shared_ptr<spdlog::logger> logger_;  ///> logger

  std::unique_ptr<Spp> rover_, base_;  ///> rover server and base server

  utils::RingBuffer<PvtSolutionRecord> solution_;  ///> solution
};

class NAVP_EXPORT RtkServer : public Task, public Rtk {
 public:
  RtkServer(std::string_view cfg_path, bool enabled_mt = false);

  virtual ~RtkServer() noexcept = default;
};

}  // namespace navp::solution