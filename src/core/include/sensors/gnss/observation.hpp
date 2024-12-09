#pragma once

#include "io/record.hpp"
#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/gTime.hpp"
#include "utils/macro.hpp"

// forward declaration
namespace navp::io::rinex {
class RinexStream;
}

namespace navp::sensors::gnss {

// forward declaration
class TropModel;

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
  f64 code_var = 0;                ///< Variance of code measurement
  f64 phase_var = 0;               ///< Variance of phase measurement
  f64 biases[2] = {std::nan("")};  ///< bias of code measurement
  f64 bias_vars[2] = {};           ///< Variance bias of phase measurement
};

/** Raw observation data from a receiver. Not to be modified by processing functions
 */
struct GObs {
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
  using StorageType = std::map<EpochUtc, std::map<Sv, std::shared_ptr<GObs>>>;

 public:
  using ObsMap = std::map<Sv, std::shared_ptr<GObs>>;

  GnssObsRecord() = default;

  GnssObsRecord(ObsList&& _obs_list) noexcept;

  GnssObsRecord(std::unique_ptr<ObsList>&& _obs_ptr) noexcept;

  virtual ~GnssObsRecord() override;

  operator std::shared_ptr<GnssObsRecord>() noexcept;

  // get maximum storage
  i32 storage() const noexcept;

  // set maximum storage
  GnssObsRecord& set_storage(i32 storage) noexcept;

  // get gnss observation frequency
  u32 frequency() const noexcept;

  // set gnss observation frequency
  GnssObsRecord& set_frequcney(u32 frequency) noexcept;

  // merge another GnssObsRecord
  GnssObsRecord& merge_record(GnssObsRecord&& record) noexcept;

  // get the first reocrded observation time
  auto begin_time() const noexcept -> EpochUtc;

  // get the last recorded observation time
  auto end_time() const noexcept -> EpochUtc;

  // get the recorded observation time range
  auto period() const noexcept -> std::tuple<EpochUtc, EpochUtc>;

  // get the recorded observation time list
  auto epoches() const noexcept -> std::vector<EpochUtc>;

  // get available satellites at given epoch
  auto sv_at(EpochUtc time) const noexcept -> std::vector<Sv>;

  // quary gnss observation at given epoch
  auto query(EpochUtc time) const noexcept -> const ObsMap*;

  // quary gnss observation at given epoch and targeted satellite
  auto query(EpochUtc time, Sv sv) const noexcept -> const std::shared_ptr<GObs>;

  // get observation by index
  auto operator[](i64 index) const -> const ObsMap*;

  // get latest observation
  auto latest() const -> const StorageType::value_type&;

  friend class io::rinex::RinexStream;

 protected:
  // add obs list and update obs_map
  void add_obs_list(ObsList&& obs_list) noexcept;
  // erase
  void erase() noexcept;

  i32 storage_ = 5;    ///> storage_ < 0, meaning limitless
  u32 frequceny_ = 1;  ///> observation frequency
  StorageType obs_map_;
};

}  // namespace navp::sensors::gnss