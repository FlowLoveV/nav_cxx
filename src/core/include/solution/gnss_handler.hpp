#pragma once

#include <memory>

#include "io/stream.hpp"
#include "solution/config.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisSolver;
class GnssNavRecord;
class GnssObsRecord;
class TropModel;
class IonoModel;
class GnssCorrections;
class Sv;
struct Sig;

}  // namespace navp::sensors::gnss

namespace navp::solution {

using navp::sensors::gnss::EphemerisSolver;
using navp::sensors::gnss::GnssCorrections;
using navp::sensors::gnss::GnssNavRecord;
using navp::sensors::gnss::GnssObsRecord;
using navp::sensors::gnss::IonoModel;
using navp::sensors::gnss::Sig;
using navp::sensors::gnss::Sv;
using navp::sensors::gnss::TropModel;

class GnssHandler;

class NAVP_EXPORT GnssHandler {
 public:
  void initialize(const NavConfigManger& config, bool is_rover = true);

  // EphemerisSolver& ephemeris_solver() noexcept;

  // GnssObsRecord& observation() noexcept;

  // TropModel& trop_model() noexcept;

  // read next observation, return true if success
  bool load_next_observation() noexcept;

  // get available satellites vector at latest epoch
  auto available_satellites() noexcept -> std::vector<Sv>;

  // generate a gnss corrections handler to get corrections related to target satellite
  auto generate_corrections_handler(EpochUtc time, const utils::CoordinateXyz& station_pos) noexcept
      -> std::function<GnssCorrections(Sv)>;

  // generate a gnss random model handler to get variance of pseudorange and carrier
  auto generate_random_handler(EpochUtc time) noexcept -> std::function<const Sig*(Sv, ObsCodeEnum)>;

 protected:
  /*
   * Gnss navigation and observation
   */
  std::vector<GnssNavRecord> nav_;               ///> record of gnss navigation
  std::unique_ptr<EphemerisSolver> eph_solver_;  ///> broadcast ephemeris solver
  std::unique_ptr<GnssObsRecord> obs_;           ///> record of gnss observation
  std::unique_ptr<io::Stream> obs_stream_;       ///> obs stream

  /*
   * Observation Type
   */
  NavConfigManger::CodeTypeMap obs_code_;

  /*
   * Atmospheric model
   */
  std::unique_ptr<TropModel> trop_model_;  ///> trop model
  std::unique_ptr<IonoModel> iono_model_;  ///> iono model

  /*
   * Random model
   */
  RandomModelEnum random_model_;

  /*
   * Solution mode
   */
  SolutionModeEnum solution_mode_;

  std::shared_ptr<spdlog::logger> logger_;  ///> handler logger

 private:
  void initialize_navigation(const NavConfigManger& config, bool is_rover = true);
  void initialize_observation(const NavConfigManger& config, bool is_rover = true);
  void initialize_atmosphere_model(const NavConfigManger& config);
  void initialize_obs_code(const NavConfigManger& config);
  void initialize_random_model(const NavConfigManger& config);
  void initialize_solution_mode(const NavConfigManger& config);
  void initialize_logger(const NavConfigManger& config);
};

}  // namespace navp::solution