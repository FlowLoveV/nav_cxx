#pragma once

#include <Eigen/Eigen>

#include "utils/eigen.hpp"
#include "utils/gTime.hpp"

namespace navp::ginan {

struct AttStatus {
  utils::GTime startTime =
      utils::GTime::noTime();  ///< Time of switchover to modified yaw steering (due to noon/midnight turn)
  f64 startSign = 0;           ///< Sign of yaw rate at switchover
  f64 startYaw = 0;            ///< Yaw at switchover
  f64 startYawRate = 0;        ///< Yaw rate at switchover
  utils::GTime excludeTime =
      utils::GTime::noTime();  ///< Time to skip yaw modelling until, due to unknown yaw behaviour

  f64 nominalYaw = 0;                                  ///< Latest nominal yaw
  f64 modelYaw = 0;                                    ///< Latest modelled yaw (i.e. considering noon/midnight turns)
  utils::GTime modelYawTime = utils::GTime::noTime();  ///< Time of modelYaw (and nominalYaw)
  bool modelYawValid = false;                          ///< Model yaw was calculated sucessfully

  utils::NavVector3f64 eXBody;  ///< X+ unit vector of body-fixed coordinates (ECEF)
  utils::NavVector3f64 eYBody;  ///< Y+ unit vector of body-fixed coordinates (ECEF)
  utils::NavVector3f64 eZBody;  ///< Z+ unit vector of body-fixed coordinates (ECEF)

  utils::NavVector3f64 eXAnt;  ///< X+ unit vector of antenna-fixed coordinates (ECEF)
  utils::NavVector3f64 eYAnt;  ///< Y+ unit vector of antenna-fixed coordinates (ECEF)
  utils::NavVector3f64 eZAnt;  ///< Z+ unit vector of antenna-fixed coordinates (ECEF)
};

}  // namespace navp::ginan