#pragma once

#include <memory>

#include "io/stream.hpp"
#include "sensors/gnss/analysis.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/config.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisSolver;
class GnssNavRecord;
class GnssObsRecord;
class TropHandler;
class IonoHandler;
class Sv;
struct Sig;

struct GnssStationInfo;
struct GnssRuntimeInfo;
struct GnssRecord;
struct GnssSettings;
struct UnDiffObsHandler;
struct GnssStationHandler;

using ClockParameterMap = std::unordered_map<ConstellationEnum, u8>;
using AtmosphereHandlerMap = std::unordered_map<Sv, std::function<f64(const utils::CoordinateBlh* station_pos)>>;
using CodeMap = std::unordered_map<ConstellationEnum, std::unordered_set<ObsCodeEnum>>;

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
  std::unique_ptr<io::Stream> obs_stream;       // obs stream

  // update gnss record and return the new runtime info
  // return true if update succeed, false if not
  bool update(GnssRuntimeInfo* runtime_info);
};

struct NAVP_EXPORT GnssSettings {
  TropModelEnum trop;                            // trop model
  IonoModelEnum iono;                            // iono model
  RandomModelEnum random;                        // random model
  i32 capacity;                                  // observation capacity
  std::unique_ptr<CodeMap> enabled_obs_code;     // enabled observation Type Map, when it is empty,
                                                 // meaning > all system and codes are defaultly enabled
  std::unique_ptr<ClockParameterMap> clock_map;  // clock parameter map

  inline bool enabled(Sv sv, const Sig& sig) const noexcept {
    return enabled(sv) && enabled_codes(sv).contains(sig.code);
  }

  inline bool enabled(Sv sv) const noexcept {
    return enabled_obs_code->empty() || enabled_obs_code->contains(sv.system());
  }

  inline auto enabled_codes(Sv sv) const noexcept -> const std::unordered_set<ObsCodeEnum>& {
    return enabled_obs_code->at(sv.system());
  };

  inline auto clock_index(Sv sv) const noexcept { return clock_map->at(sv.system()); }
};

struct NAVP_EXPORT RawObsHandler {
  const GObs* obs;
  std::vector<const Sig*> sig;
};

struct NAVP_EXPORT UnDiffObsHandler {
  bool trop_corr;
  bool iono_corr;
  CombObsMeta meta;
  const GObs* obs;
};

class NAVP_EXPORT GnssStationHandler {
  friend class navp::GlobalConfig;

 public:
  GnssStationHandler() noexcept = default;

  GnssStationHandler(const GnssStationHandler& handler) noexcept = delete;
  GnssStationHandler& operator=(const GnssStationHandler& handler) noexcept = delete;

  GnssStationHandler(GnssStationHandler&& handler) noexcept = default;
  GnssStationHandler& operator=(GnssStationHandler&& handler) noexcept = default;

  virtual ~GnssStationHandler() = default;

  /* station information */
  inline auto station_info() const noexcept -> const GnssStationInfo* { return station_info_.get(); }
  inline auto station_info() noexcept -> GnssStationInfo* { return const_cast<GnssStationInfo*>(station_info_.get()); }

  /* runtime information */
  inline auto runtime_info() const noexcept -> const GnssRuntimeInfo* { return runtime_info_.get(); }
  inline auto runtime_info() noexcept -> GnssRuntimeInfo* { return const_cast<GnssRuntimeInfo*>(runtime_info_.get()); }

  /* station settings */
  inline auto settings() const noexcept -> const GnssSettings* { return settings_.get(); }
  inline auto runtime_settings() noexcept -> GnssSettings* { return const_cast<GnssSettings*>(settings_.get()); }

  /* station record */
  inline auto record() const noexcept -> const GnssRecord* { return record_.get(); }
  inline auto record() noexcept -> GnssRecord* { return const_cast<GnssRecord*>(record_.get()); }

  /* station logger */
  inline auto logger() const noexcept -> const std::shared_ptr<spdlog::logger>& { return logger_; }

  // update gnss station record
  virtual inline bool update_record() { return record_->update(runtime_info_.get()); }

  // update runtime information
  virtual inline void update_runtime_info() { runtime_info_->update(record_.get()); }

  // generate a vector of RawObsHandler
  // # usage
  // - Spp
  // - Analysis
  NAV_NODISCARD_UNUNSED auto generate_rawobs_handler() const noexcept -> std::vector<RawObsHandler>;

  NAV_NODISCARD_UNUNSED auto generate_undiffobs_handler() const noexcept -> std::vector<UnDiffObsHandler>;

  // generate a gnss trop handler to get trop corrections
  // - examples
  // - auto trop_handler = gnss_handler.generate_trop_handler(sv);
  // - auto trop_corrections = trop_handler(station_pos);
  NAV_NODISCARD_UNUNSED auto generate_trop_handler(Sv sv) const noexcept
      -> std::function<f64(const utils::CoordinateBlh*)>;

  // NAV_NODISCARD_UNUNSED auto generate_trop_handler() const noexcept -> AtmosphereHandlerMap;

  // generate a gnss trop handler to get trop corrections
  // - examples
  // - auto iono_handler = gnss_handler.generate_iono_handler(sv);
  // - auto iono_corrections = iono_handler(station_pos);
  NAV_NODISCARD_UNUNSED auto generate_iono_handler(Sv sv) const noexcept
      -> std::function<f64(const utils::CoordinateBlh*)>;

  // NAV_NODISCARD_UNUNSED auto generate_iono_handler() const noexcept -> AtmosphereHandlerMap;

  // generate a gnss random model handler to get variance of pseudorange and carrier
  // - examples
  // - auto random_handler = gnss_handler.generate_random_handler(sv);
  // - auto random_corrections = random_handler(ObsCodeEnum::C1C);
  NAV_NODISCARD_UNUNSED auto generate_random_handler(Sv sv) const noexcept -> std::function<const Sig*(ObsCodeEnum)>;

 protected:
  std::unique_ptr<GnssStationInfo> station_info_;  // station info

  std::unique_ptr<GnssRuntimeInfo> runtime_info_;  // runtime info

  std::unique_ptr<GnssRecord> record_;  // storage

  std::unique_ptr<GnssSettings> settings_;  // settings

  std::shared_ptr<spdlog::logger> logger_;  // handler logger
};

}  // namespace navp::sensors::gnss