#pragma once

#include "io/record.hpp"
#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/option.hpp"
#include "utils/time.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

using navp::Epoch;

class NAVP_EXPORT GnssObsRecord;
struct NAVP_EXPORT GObs;

/** Raw observation data from a receiver for a single frequency. Not to be modified by processing functions
 */
struct NAVP_EXPORT RawSig {
  ObsCodeEnum code = ObsCodeEnum::NONE;  ///< Reported code type
  bool LLI = false;                      ///< Loss of lock indicator
  bool invalid = false;

  f64 L = 0;    ///< Carrier phase (cycles)
  f64 P = 0;    ///< Pseudorange (meters)
  f64 D = 0;    ///< Doppler
  f64 snr = 0;  ///< Signal to Noise ratio (dB-Hz)

  bool operator<(const RawSig& b) const { return (code < b.code); }
};

/** Per signal data that is calculated from the raw signals.
 */
struct NAVP_EXPORT Sig : RawSig {
  f64 codeVar = 0;                 ///< Variance of code measurement
  f64 phasVar = 0;                 ///< Variance of phase measurement
  f64 biases[2] = {std::nan("")};  ///< bias of code measurement
  f64 biasVars[2] = {};            ///< Variance bias of phase measurement
};

/** Raw observation data from a receiver. Not to be modified by processing functions
 */
struct GObs {
  // std::map<FreTypeEnum, Sig> sigs;  ///> std::map of signals available in this observation (one per frequency only)
  std::map<FreTypeEnum, std::list<Sig>> sigsLists;  ///> std::map of all signals available in this observation (may
                                                    /// include multiple per frequency, eg L1X, L1C)
  Sv Sat = {};                                      ///> Satellite ID (system, prn)
  utils::GTime time = {};                           ///< Receiver sampling time (GPST)

  const Sig* find_code(ObsCodeEnum code) const noexcept;

  operator std::shared_ptr<GObs>();
};

/** List of observations for an epoch
 */
struct NAVP_EXPORT ObsList : std::vector<std::shared_ptr<GObs>> {
  ObsList& operator+=(const ObsList& right);
};

class GnssObsRecord : public io::Record {
 public:
  GnssObsRecord() = default;
  GnssObsRecord(ObsList&& _obs_list) noexcept;
  GnssObsRecord(std::unique_ptr<ObsList>&& _obs_ptr) noexcept;

  virtual ~GnssObsRecord() override;

  operator std::shared_ptr<GnssObsRecord>() noexcept;

  // add obs list and update obs_map
  void add_obs_list(ObsList&& obs_list) noexcept;

  // add another obs_map and update obs_map
  void add_record(GnssObsRecord&& record) noexcept;

  // analysis observation time
  auto begin_time() const noexcept -> EpochUtc;
  auto end_time() const noexcept -> EpochUtc;
  auto period() const noexcept -> std::tuple<EpochUtc, EpochUtc>;
  auto epoches() const noexcept -> std::vector<EpochUtc>;

  // analysis sv at target time
  auto sv(EpochUtc time) const noexcept -> Option<std::vector<Sv>>;

  // get target observation
  auto query(EpochUtc time) const noexcept -> Option<std::map<Sv, std::shared_ptr<GObs>>>;
  auto query(EpochUtc time, Sv sv) const noexcept -> Option<std::shared_ptr<GObs>>;

 protected:
  std::map<EpochUtc, std::map<Sv, std::shared_ptr<GObs>>> obs_map;
};

}  // namespace navp::sensors::gnss