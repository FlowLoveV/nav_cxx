#pragma once

#include <list>
#include <unordered_set>

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

class GnssObsRecord;
struct GObs;

using CodeMap = std::unordered_map<ConstellationEnum, std::unordered_set<ObsCodeEnum>>;

/** Raw observation data from a receiver for a single frequency. Not to be modified by processing functions
 */
struct NAVP_EXPORT RawSig {
  enum ValidIndicator : u8 {
    Valid = 0,
    CycleSlip = 1,
  };

  ObsCodeEnum code = ObsCodeEnum::NONE;          // Reported code type
  FreTypeEnum freq = FreTypeEnum::FTYPE_NONE;    // Frequency type
  bool lli = false;                              // Loss of lock indicator
  ValidIndicator valid = ValidIndicator::Valid;  // invaild indicator
  f32 snr = 0;                                   // Signal to Noise ratio (dB-Hz)
  f32 doppler = 0;                               // Doppler
  f64 carrier = 0;                               // Carrier phase (cycles)
  f64 pseudorange = 0;                           // Pseudorange (meters)

  inline bool operator<(const RawSig& b) const { return (code < b.code); }

  inline bool is_valid() const noexcept { return valid == Valid; }

  inline bool is_cycle_slip() const noexcept { return valid == CycleSlip; }
};

/** Per signal data that is calculated from the raw signals.
 */
struct NAVP_EXPORT Sig : RawSig {
  f64 code_var = 0;                // Variance of code measurement
  f64 phase_var = 0;               // Variance of phase measurement
  f64 biases[2] = {std::nan("")};  // bias of code measurement
  f64 bias_vars[2] = {};           // Variance bias of phase measurement
};

/** Raw observation data from a receiver. Not to be modified by processing functions
 */
struct NAVP_EXPORT GObs {
  std::unordered_map<FreTypeEnum, std::list<Sig>>
      sigs_list;           ///> std::map of all signals available in this observation (may
                           /// include multiple per frequency, eg L1X, L1C)
  Sv sv = {};              ///> Satellite ID (system, prn)
  utils::GTime time = {};  ///> Receiver sampling time (GPST)

  const Sig* find_code(ObsCodeEnum code) const noexcept;

  u8 frequency_count() const noexcept;

  u8 code_count() const noexcept;

  template <typename Func>
  void for_each_frequency(Func&& func) {
    for (auto&& [_, sigs] : sigs_list) {
      std::invoke(std::forward<Func>(func), std::forward<std::list<Sig>>(sigs));
    }
  }

  template <typename Func>
  void for_each_code(Func&& func) {
    for (auto&& [_, sigs] : sigs_list) {
      for (auto&& sig : sigs) {
        std::invoke(std::forward<Func>(func), std::forward<Sig>(sig));
      }
    }
  }

  void check_vaild() noexcept;
};

/** List of observations for an epoch
 */
struct NAVP_EXPORT ObsList : std::vector<std::shared_ptr<GObs>> {
  ObsList& operator+=(const ObsList& right);
};

class NAVP_EXPORT GnssObsRecord : public io::Record {
  using StorageType = std::map<EpochUtc, std::unordered_map<Sv, std::shared_ptr<GObs>>>;

 public:
  using ObsMap = std::unordered_map<Sv, std::shared_ptr<GObs>>;
  using ObsPtr = std::shared_ptr<GObs>;

  GnssObsRecord(std::shared_ptr<spdlog::logger> logger);

  virtual ~GnssObsRecord() override;

  // code map
  const CodeMap& code_map() const noexcept;

  // get maximum storage
  i32 storage() const noexcept;

  // set maximum storage
  GnssObsRecord& set_storage(i32 storage) noexcept;

  // get gnss observation frequency
  u32 frequency() const noexcept;

  // set gnss observation frequency
  GnssObsRecord& set_frequency(u32 frequency) noexcept;

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
  auto at(EpochUtc time) const noexcept -> const ObsMap*;

  // quary gnss observation at given epoch and targeted satellite
  auto at(EpochUtc time, Sv sv) const noexcept -> const GObs*;

  // check if contains an epoch
  auto contains(EpochUtc time) const noexcept -> bool;

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

  i32 storage_ = -1;                        // observation storage, when storage_ < 0, meaning limitless
  u32 frequceny_ = 1;                       // observation frequency
  char glo_fcn_[27 + 1];                    // glonass frequency channel number + 8
  f64 glo_cpbias_[4];                       // glonass code-phase bias {1C,1P,2C,2P} (m)
  CodeMap code_map_;                        // observation code map
  StorageType obs_map_;                     // observation map
  std::shared_ptr<spdlog::logger> logger_;  // logger
};

}  // namespace navp::sensors::gnss