/// fixed sized float matrix based eigen

#pragma once

#include <Eigen/Eigen>

#include "utils/types.hpp"

namespace navp::utils {

template <size_t Row, size_t Col>
using NavMatrixf128 = Eigen::Matrix<f128, Row, Col>;
template <size_t Row, size_t Col>
using NavMatrixf64 = Eigen::Matrix<f64, Row, Col>;
template <size_t Row, size_t Col>
using NavMatrixf32 = Eigen::Matrix<f32, Row, Col>;
template <size_t Row>
using NavVectorf32 = Eigen::Vector<f32, Row>;
template <size_t Row>
using NavVectorf64 = Eigen::Vector<f64, Row>;

template <size_t Row>
using NavMatrixDcf64 = Eigen::Matrix<f64, Row, Eigen::Dynamic>;
template <size_t Row>
using NavMatrixDcf32 = Eigen::Matrix<f32, Row, Eigen::Dynamic>;
template <size_t Col>
using NavMatrixDrf64 = Eigen::Matrix<f64, Eigen::Dynamic, Col>;
template <size_t Col>
using NavMatrixDrf32 = Eigen::Matrix<f32, Eigen::Dynamic, Col>;

using NavMatrixDf64 = Eigen::Matrix<f64, Eigen::Dynamic, Eigen::Dynamic>;
using NavMatrixDf32 = Eigen::Matrix<f32, Eigen::Dynamic, Eigen::Dynamic>;
using NavVectorDf32 = Eigen::Vector<f32, Eigen::Dynamic>;
using NavVectorDf64 = Eigen::Vector<f64, Eigen::Dynamic>;

using NavVector2f64 = NavMatrixf64<2, 1>;
using NavVector3f64 = NavMatrixf64<3, 1>;
using NavVector4f64 = NavMatrixf64<4, 1>;
using NavVector5f64 = NavMatrixf64<5, 1>;
using NavVector6f64 = NavMatrixf64<6, 1>;

using NavVector2f32 = NavMatrixf32<2, 1>;
using NavVector3f32 = NavMatrixf32<3, 1>;
using NavVector4f32 = NavMatrixf32<4, 1>;
using NavVector5f32 = NavMatrixf32<5, 1>;
using NavVector6f32 = NavMatrixf32<6, 1>;

using NavMatrix22f64 = NavMatrixf64<2, 2>;
using NavMatrix23f64 = NavMatrixf64<2, 3>;
using NavMatrix24f64 = NavMatrixf64<2, 4>;
using NavMatrix25f64 = NavMatrixf64<2, 5>;
using NavMatrix26f64 = NavMatrixf64<2, 6>;

using NavMatrix32f64 = NavMatrixf64<3, 2>;
using NavMatrix33f64 = NavMatrixf64<3, 3>;
using NavMatrix34f64 = NavMatrixf64<3, 4>;
using NavMatrix35f64 = NavMatrixf64<3, 5>;
using NavMatrix36f64 = NavMatrixf64<3, 6>;

using NavMatrix42f64 = NavMatrixf64<4, 2>;
using NavMatrix43f64 = NavMatrixf64<4, 3>;
using NavMatrix44f64 = NavMatrixf64<4, 4>;
using NavMatrix45f64 = NavMatrixf64<4, 5>;
using NavMatrix46f64 = NavMatrixf64<4, 6>;

using NavMatrix52f64 = NavMatrixf64<5, 2>;
using NavMatrix53f64 = NavMatrixf64<5, 3>;
using NavMatrix54f64 = NavMatrixf64<5, 4>;
using NavMatrix55f64 = NavMatrixf64<5, 5>;
using NavMatrix56f64 = NavMatrixf64<5, 6>;

using NavMatrix62f64 = NavMatrixf64<6, 2>;
using NavMatrix63f64 = NavMatrixf64<6, 3>;
using NavMatrix64f64 = NavMatrixf64<6, 4>;
using NavMatrix65f64 = NavMatrixf64<6, 5>;
using NavMatrix66f64 = NavMatrixf64<6, 6>;

using NavMatrix22f32 = NavMatrixf32<2, 2>;
using NavMatrix23f32 = NavMatrixf32<2, 3>;
using NavMatrix24f32 = NavMatrixf32<2, 4>;
using NavMatrix25f32 = NavMatrixf32<2, 5>;
using NavMatrix26f32 = NavMatrixf32<2, 6>;

using NavMatrix32f32 = NavMatrixf32<3, 2>;
using NavMatrix33f32 = NavMatrixf32<3, 3>;
using NavMatrix34f32 = NavMatrixf32<3, 4>;
using NavMatrix35f32 = NavMatrixf32<3, 5>;
using NavMatrix36f32 = NavMatrixf32<3, 6>;

using NavMatrix42f32 = NavMatrixf32<4, 2>;
using NavMatrix43f32 = NavMatrixf32<4, 3>;
using NavMatrix44f32 = NavMatrixf32<4, 4>;
using NavMatrix45f32 = NavMatrixf32<4, 5>;
using NavMatrix46f32 = NavMatrixf32<4, 6>;

using NavMatrix52f32 = NavMatrixf32<5, 2>;
using NavMatrix53f32 = NavMatrixf32<5, 3>;
using NavMatrix54f32 = NavMatrixf32<5, 4>;
using NavMatrix55f32 = NavMatrixf32<5, 5>;
using NavMatrix56f32 = NavMatrixf32<5, 6>;

using NavMatrix62f32 = NavMatrixf32<6, 2>;
using NavMatrix63f32 = NavMatrixf32<6, 3>;
using NavMatrix64f32 = NavMatrixf32<6, 4>;
using NavMatrix65f32 = NavMatrixf32<6, 5>;
using NavMatrix66f32 = NavMatrixf32<6, 6>;

using NavMatrix2df64 = NavMatrixDcf64<2>;
using NavMatrix3df64 = NavMatrixDcf64<3>;
using NavMatrix4df64 = NavMatrixDcf64<4>;
using NavMatrix5df64 = NavMatrixDcf64<5>;
using NavMatrix6df64 = NavMatrixDcf64<6>;

using NavMatrix2df32 = NavMatrixDcf32<2>;
using NavMatrix3df32 = NavMatrixDcf32<3>;
using NavMatrix4df32 = NavMatrixDcf32<4>;
using NavMatrix5df32 = NavMatrixDcf32<5>;
using NavMatrix6df32 = NavMatrixDcf32<6>;

using NavMatrixd2f64 = NavMatrixDrf64<2>;
using NavMatrixd3f64 = NavMatrixDrf64<3>;
using NavMatrixd4f64 = NavMatrixDrf64<4>;
using NavMatrixd5f64 = NavMatrixDrf64<5>;
using NavMatrixd6f64 = NavMatrixDrf64<6>;

using NavMatrixd2f32 = NavMatrixDrf32<2>;
using NavMatrixd3f32 = NavMatrixDrf32<3>;
using NavMatrixd4f32 = NavMatrixDrf32<4>;
using NavMatrixd5f32 = NavMatrixDrf32<5>;
using NavMatrixd6f32 = NavMatrixDrf32<6>;

using NavQuaternionf64 = Eigen::Quaternion<f64>;
using NavQuaternionf32 = Eigen::Quaternion<f32>;

using AngleAxisf64 = Eigen::AngleAxis<f64>;
using AngleAxisf32 = Eigen::AngleAxis<f32>;

}  // namespace navp::utils

#include <format>
#include "utils/ostream_formatter.hpp"

namespace std {

template <typename T>
  requires std::is_base_of_v<Eigen::DenseBase<T>, T>
struct formatter<T> : navp::utils::ostream_formatter {};

}  // namespace std