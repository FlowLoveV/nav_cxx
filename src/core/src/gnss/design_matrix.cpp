#include "sensors/gnss/design_matrix.hpp"

#include <ranges>

#include "sensors/gnss/analysis.hpp"
#include "sensors/gnss/broadcast_eph.hpp"
#include "utils/option.hpp"

namespace navp::sensors::gnss {

auto DesignMatrixBuilder::constellation() const noexcept -> std::map<Constellation, u8> {
  std::map<Constellation, u8> res;
  u8 index = 0;
  for (auto sv : *sv_info | std::views::keys) {
    if (!res.contains(sv.constellation)) {
      res.insert({sv.constellation, index++});
    }
  }
  return res;
}

Option<DesignMatrix> DesignMatrixBuilder::spp_design_matrix(const MetaVec& meta_vec,
                                                            utils::NavVector3f64 station_xyz) const noexcept {
  auto cons = constellation();
  auto row = meta_vec.size();
  auto col = cons.size() + 3;
  auto res = DesignMatrix{
      .matrix = utils::NavMatrixDf64::Zero(row, col),
  };
  // assign design matrix
  for (auto index = 0; index < row; index++) {
    auto sv = meta_vec[index].first;
    auto meta = meta_vec[index].second;
    if (!sv_info->contains(sv)) {
      nav_error("{} is not usable!", sv);
      return None;
    }
    if (meta.is_difference_obs()) {
      nav_error("can't use difference observation in spp!");
      return None;
    }
    auto eph_res = sv_info->at(sv);
    auto row0 = (eph_res.pos - station_xyz).norm();
    // assign position
    res.matrix.block(index, 0, 1, 3) = (station_xyz - eph_res.pos) / row0;
    // assign receiver clock bias
    res.matrix(index, cons[sv.constellation] + 3) = 1;
  }

  return {std::move(res)};
}

}  // namespace navp::sensors::gnss