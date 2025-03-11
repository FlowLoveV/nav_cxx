#pragma once

#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::algorithm {

class NAVP_EXPORT AmbiguityFixer {
 public:
  AmbiguityFixer(utils::NavVector3f64 float_baseline, f64* float_ambiguity, const utils::NavMatrixDf64& float_qxx);

  bool fix() noexcept;

  inline f64 ratio() const noexcept { return ratio_; }

  inline auto fixed_baseline() const noexcept -> const utils::NavVector3f64& { return fixed_baseline_; }

  inline auto fixed_ambiguity() const noexcept -> const utils::NavVectorDf64& { return fixed_ambiguity_; }

  ~AmbiguityFixer() = default;

 private:
  bool _valid() const noexcept;

  utils::NavMatrixDf64 _construct_k1() const noexcept;  // a = K1 * x
  utils::NavMatrixDf64 _construct_k2() const noexcept;  // b = K2 * x

  enum class State : u8 {
    InitializeSucceed = 1,
    InitializeFail = 2,
    FixSuccess = 3,
    FixFail = 4,
  } state_{1};

  f32 ratio_{1.0f};
  utils::NavVector3f64 float_baseline_, fixed_baseline_;
  Eigen::Map<utils::NavVectorDf64> float_ambiguity_;
  utils::NavVectorDf64 fixed_ambiguity_;
  const utils::NavMatrixDf64* float_qxx_;
  utils::NavMatrix33f64 float_qbb_, fixed_qbb_;
};

}  // namespace navp::algorithm
