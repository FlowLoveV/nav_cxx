#pragma once

#include <Eigen/Eigen>

#include "enums.hpp"
#include "sv.hpp"
#include "utils/gTime.hpp"
#include "utils/option.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

using navp::Epoch;

class RecordGnssObs;
struct GObs;

struct Observation {
  virtual ~Observation() = default;

 protected:
  GObs* gObs_ptr = nullptr;
};

/** Raw observation data from a receiver for a single frequency. Not to be modified by processing functions
 */
struct RawSig {
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
struct Sig : RawSig {
  f64 codeVar = 0;                 ///< Variance of code measurement
  f64 phasVar = 0;                 ///< Variance of phase measurement
  f64 biases[2] = {std::nan("")};  ///< bias of code measurement
  f64 biasVars[2] = {};            ///< Variance bias of phase measurement
};

/** Raw observation data from a receiver. Not to be modified by processing functions
 */
struct GObs : Observation {
  // std::map<FreTypeEnum, Sig> sigs;  ///> std::map of signals available in this observation (one per frequency only)
  std::map<FreTypeEnum, std::list<Sig>> sigsLists;  ///> std::map of all signals available in this observation (may
                                                    /// include multiple per frequency, eg L1X, L1C)
  Sv Sat = {};                                      ///> Satellite ID (system, prn)
  utils::GTime time = {};                           ///< Receiver sampling time (GPST)

  operator std::shared_ptr<GObs>() {
    auto pointer = std::make_shared<GObs>(*this);

    pointer->gObs_ptr = pointer.get();

    return pointer;
  }

  virtual ~GObs() = default;
};

/** List of observations for an epoch
 */
struct ObsList : std::vector<std::shared_ptr<Observation>> {
  ObsList& operator+=(const ObsList& right) {
    this->insert(this->end(), right.begin(), right.end());
    return *this;
  }
};

class RecordGnssObs {
 public:
  RecordGnssObs() = default;
  RecordGnssObs(ObsList&& _obs_list) noexcept;
  RecordGnssObs(std::unique_ptr<ObsList>&& _obs_ptr) noexcept;

  std::shared_ptr<ObsList> obs_list;

  operator std::shared_ptr<RecordGnssObs>() noexcept;


  // analysis observation time
  auto begin_time() const noexcept -> Epoch<UTC>;
  auto end_time() const noexcept -> Epoch<UTC>;
  auto period() const noexcept -> std::tuple<Epoch<UTC>, Epoch<UTC>>;
  auto epoches() const noexcept -> std::vector<Epoch<UTC>>;

  // analysis sv at target time
  auto sv(Epoch<UTC> time) const noexcept -> Option<std::vector<Sv>>;

  // get target observation
  auto query(Epoch<UTC> time) const noexcept -> Option<std::map<Sv, std::map<FreTypeEnum, std::list<Sig>>>>;
  auto query(Epoch<UTC> time, Sv sv) const noexcept -> Option<std::map<FreTypeEnum, std::list<Sig>>>;
  
 protected:
  // generate obs map from obs_list
  void generate_obs_map() const noexcept;

  mutable std::map<Epoch<UTC>, std::map<Sv, std::map<FreTypeEnum, std::list<Sig>>>> obs_map;
};

}  // namespace navp::sensors::gnss