#pragma once

#include "filter/filter.hpp"
#include "io/stream.hpp"
#include "sensors/gnss/analysis.hpp"
#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/observation.hpp"
#include "sensors/gnss/random.hpp"
#include "solution/config.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

class AtmosphereHandler;

struct GnssStationInfo;
struct GnssRuntimeInfo;
struct GnssRecord;
struct GnssSettings;
struct UnDiffObsHandler;
struct GnssPayload;

using ClockParameterMap = std::unordered_map<ConstellationEnum, u8>;
using AtmosphereHandlerMap = std::unordered_map<Sv, AtmosphereHandler>;
using RandomHandlerMap = std::unordered_map<Sv, GnssRandomHandler>;

struct NAVP_EXPORT GnssStationInfo {
  u8 type;                                        // 0: rover, 1: base
  bool fixed;                                     // true: fixed, false: moveable
  u8 source;                                      // 0: file , 1: network,  2: serial port
  u8 healthy;                                     // 0: healthy, 1: unhealthy
  u32 frequency;                                  // observation frequency
  std::unique_ptr<utils::CoordinateXyz> ref_pos;  // reference position XYZ(if fixed)
  std::string name;                               // station name
};

struct NAVP_EXPORT GnssRuntimeInfo {
  EpochUtc epoch;                        // latest observation epoch
  const GnssObsRecord::ObsMap* obs_map;  // latest observation
  const EphemerisSolver::SvMap* sv_map;  // latest sv map
  std::vector<Sv> avilable_sv;           // latest available satellite vector

  void update(const GnssRecord* record);
};

struct NAVP_EXPORT GnssRecord {
  std::list<GnssNavRecord> nav;                 // record of gnss navigation
  std::unique_ptr<EphemerisSolver> eph_solver;  // ephemeris solver
  std::unique_ptr<GnssObsRecord> obs;           // record of gnss observation
  std::unique_ptr<io::Fstream> obs_stream;      // obs stream

  // return true if update observation succeed, false if not
  bool update();
};

struct NAVP_EXPORT GnssSettings {
  TropModelEnum trop;                         // trop model
  IonoModelEnum iono;                         // iono model
  RandomModelEnum random;                     // random model
  i32 capacity;                               // observation capacity
  std::unique_ptr<CodeMap> enabled_obs_code;  // enabled observation Type Map, when it is empty,
                                              // meaning > all system and codes are defaultly enabled

  bool enabled(Sv sv, const Sig& sig) const noexcept {
    return enabled_obs_code->empty() ||
           (enabled_obs_code->contains(sv.system()) && enabled_obs_code->at(sv.system()).contains(sig.code));
  }

  bool enabled(Sv sv) const noexcept { return enabled_obs_code->empty() || enabled_obs_code->contains(sv.system()); }

  auto enabled_codes(Sv sv) const noexcept -> const std::unordered_set<ObsCodeEnum>& {
    return enabled_obs_code->at(sv.system());
  };
};

// - Gnss Raw Observation Handler
// - Each instance records the signal of a single satellite
struct NAVP_EXPORT GnssRawObsHandler {
  const GObs* obs;                 // observation
  const EphemerisResult* sv_info;  // satellite information
  std::vector<const Sig*> sig;     // sigs vector

  void handle_signal_variance(RandomModelEnum model, GnssRandomHandler::EvaluateRandomOptions options =
                                                         GnssRandomHandler::Pseudorange) const noexcept;

  f64 trop_corr(const utils::CoordinateBlh* station_pos, TropModelEnum model) const noexcept;

  f64 iono_corr(const utils::CoordinateBlh* station_pos, IonoModelEnum model) const noexcept;
};

// todo
// Gnss un-difference Observation Handler
struct NAVP_EXPORT UnDiffObsHandler {
  CombObsMeta meta;
  const GObs* obs;
};

// - Gnss Double-Difference Observation Handler
// - Each instance records the double-difference observations of a single system

class GnssPayload {
  friend class navp::GlobalConfig;

 public:
  GnssPayload() noexcept = default;

  GnssPayload(const GnssPayload& handler) noexcept = delete;
  GnssPayload& operator=(const GnssPayload& handler) noexcept = delete;

  GnssPayload(GnssPayload&& handler) noexcept = default;
  GnssPayload& operator=(GnssPayload&& handler) noexcept = default;

  virtual ~GnssPayload() = default;

 protected:
  inline bool update_record() { return record_->update(); }

  inline void update_runtime_info() { runtime_info_->update(record_.get()); }

  NAV_NODISCARD_UNUNSED auto generate_rawobs_handler(const filter::MaskFilters* mask_filter = nullptr) const
      -> std::vector<GnssRawObsHandler>;

  NAV_NODISCARD_UNUNSED auto generate_undiffobs_handler() const -> std::vector<UnDiffObsHandler>;

  NAV_NODISCARD_UNUNSED auto generate_atmosphere_handler(Sv sv) const -> AtmosphereHandler;

  NAV_NODISCARD_UNUNSED auto generate_random_handler(Sv sv) const -> GnssRandomHandler;

  void decode_header(io::Fstream& stream) const noexcept;

  std::unique_ptr<GnssStationInfo> station_info_;  // station info

  std::unique_ptr<GnssRuntimeInfo> runtime_info_;  // runtime info

  std::unique_ptr<GnssRecord> record_;  // storage

  std::unique_ptr<GnssSettings> settings_;  // settings

  std::shared_ptr<spdlog::logger> logger_;  // handler logger
};

}  // namespace navp::sensors::gnss