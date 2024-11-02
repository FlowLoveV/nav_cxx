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
  char _type;
  ConstellationEnum _sys;
  TimeSystemEnum _tsys;
  f64 _version;
  std::map<ConstellationEnum, std::map<i32, CodeType>> _sys_code_types;
  std::shared_ptr<RinexStation> _station;
  std::shared_ptr<Navigation> _nav;
};

}  // namespace navp::io::rinex