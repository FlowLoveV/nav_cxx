#pragma once

#include <memory>

#include "io/rinex/rinex_reader.hpp"
#include "utils/macro.hpp"

namespace navp::io::rinex {

class RinexRecord;

class NAVP_EXPORT RinexRecord {
 public:
  RinexRecord();
  ~RinexRecord();

  /// get version of rinex file
  double version() const noexcept;

  /// get system of rinex file
  ConstellationEnum system() const noexcept;

  /// get time system of rinex file
  TimeSystemEnum time_system() const noexcept;

  /// judge type
  bool is_obs() const noexcept;
  bool is_nav() const noexcept;
  bool is_clk() const noexcept;

 protected:
  char type_;                                                            ///> file type
  ConstellationEnum sys_;                                                ///> system
  TimeSystemEnum tsys_;                                                  ///> time system
  char glo_fcn_[27 + 1];                                                 ///> glonass frequency channel number + 8
  f64 glo_cpbias_[4];                                                    ///> glonass code-phase bias {1C,1P,2C,2P} (m)
  f64 version_;                                                          ///> rinex version
  std::map<ConstellationEnum, std::map<i32, CodeType>> sys_code_types_;  ///> system code types
  std::unique_ptr<RinexStation> station_;                                ///> station info
  std::unique_ptr<Navigation> nav_;                                      ///> navigation info
};

}  // namespace navp::io::rinex