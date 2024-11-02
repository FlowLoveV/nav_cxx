#pragma once

#include "sensors/gnss/sv.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

// forward declaration
struct BrdcEphResult;
struct ObsMeta;

struct NAVP_EXPORT DesignMatrix;
class NAVP_EXPORT DesignMatrixBuilder;

typedef std::vector<std::pair<Sv, ObsMeta>> MetaVec;

struct DesignMatrix {
  utils::NavMatrixDf64 matrix;
};

class DesignMatrixBuilder {
  const std::map<Sv, BrdcEphResult>* sv_info;

  Option<DesignMatrix> spp_design_matrix(const MetaVec& meta_vec, utils::NavVector3f64 station_xyz) const noexcept;

  // todo
  // for rtk design matrix,maybe need a special class to generate and process
  DesignMatrix rtk_design_matrix() const noexcept;

 protected:
  auto constellation() const noexcept -> std::map<Constellation, u8>;
};

}  // namespace navp::sensors::gnss