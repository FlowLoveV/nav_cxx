#pragma once

#include "sensors/gnss/sv.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisSolver;
struct EphemerisResult;

using utils::NavVector3f64;

class AtmosphereModel;
class TropModel;
class IonoModel;

class NAVP_EXPORT AtmosphereModel {
 public:
  AtmosphereModel& set_time(EpochUtc tr) noexcept;

  AtmosphereModel& set_sv_status(const EphemerisSolver* solver) noexcept;

  AtmosphereModel& set_sv_status(const std::map<Sv, EphemerisResult>* eph_result) noexcept;

  AtmosphereModel& set_station_pos(const utils::CoordinateBlh* pos) noexcept;

  virtual f64 solve(Sv sv) = 0;

  virtual ~AtmosphereModel();

 protected:
  bool solvable() const noexcept;

  const utils::CoordinateBlh* station_pos_{};
  EpochUtc tr_{};
  const std::map<Sv, EphemerisResult>* sv_map_{};
};

class NAVP_EXPORT TropModel : public AtmosphereModel {
 public:
  TropModel& set_model(TropModelEnum type) noexcept;

  virtual f64 solve(Sv sv) override;

  virtual ~TropModel() override;

 protected:
  TropModelEnum type_;
};

class NAVP_EXPORT IonoModel : public AtmosphereModel {
 public:
  IonoModel& set_model(IonoModelEnum type) noexcept;

  virtual f64 solve(Sv sv) override;

  virtual ~IonoModel() override;

 protected:
  IonoModelEnum type_;
};

}  // namespace navp::sensors::gnss