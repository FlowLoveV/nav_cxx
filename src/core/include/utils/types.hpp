#pragma once

#include <cmath>
#include <memory>
#include <stdfloat>

namespace navp {

template <typename T>
using Ref = const T&;
template <typename T>
using MutRef = T&;
template <typename T>
using ForwardRef = T&&;
template <typename T>
using constForwardRef = const T&&;

template <typename T>
using RefWrapper = std::reference_wrapper<T>;
template <typename T>
inline RefWrapper<T> RefWarp(Ref<T> t) {
  return std::ref(t);
}

template <typename T>
using Uptr = std::unique_ptr<T>;
template <typename T>
using Sptr = std::shared_ptr<T>;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

#ifdef __STDCPP_FLOAT16_T__
typedef std::float16_t f16;
#endif
typedef std::float_t f32;
typedef std::double_t f64;
#ifdef __STDCPP_FLOAT128_T__
typedef std::float128_t f128;
#endif
#ifdef __STDCPP_BFLOAT16_T__
typedef std::bfloat16_t bf16;
#endif

// enum class BiographyEnum;
// enum class LliFlagEnum : u8;
// enum class EpochFlagEnum : u8;
// enum class EphemerisTypeFlag : u8;

// // biography
// enum class BiographyEnum {
//   // https://files.igs.org/pub/data/format/rinex_4.00.pdf
//   RINEX4,
// };

// enum class LliFlagEnum : u8 {
//   // Current epoch is marked Ok or Unknown status
//   OK_OR_UNKNOWN = 0x00,
//   // Lock lost between previous observation and current observation,
//   LOCK_LOSS = 0x01,
//   // Half cycle slip marker
//   HALF_CYCLE_SLIP = 0x02,
//   // Observing under anti spoofing,
//   // might suffer from decreased SNR - decreased signal quality
//   UNDER_ANTI_SPOOFING = 0x03,
// };

// enum class EpochFlagEnum : u8 {
//   /// Epoch is sane
//   Ok = 0x00,
//   /// Cycle slip at this epoch
//   CycleSlip,
//   /// Single too weak at this epoch
//   WeakSingal,
//   /// Incomplete observation at this epoch
//   IncompleteObserve,
// };

// enum class EphemerisTypeFlag : u8 {
//   /// Legacy navp message
//   LNAV,
//   /// Glonass FDMA message
//   FDMA,
//   /// Galileo FNAV message
//   FNAV,
//   /// Galileo INAV message
//   INAV,
//   /// IFNV,
//   IFNV,
//   /// BeiDou D1 navp message
//   D1,
//   /// BeiDou D2 navp message
//   D2,
//   /// D1D2
//   D1D2,
//   /// SBAS navp message
//   SBAS,
//   /// GPS / QZSS Civilian navp message
//   CNAV,
//   /// BeiDou CNV1 message
//   CNV1,
//   /// GPS / QZSS / BeiDou CNV2 message
//   CNV2,
//   /// BeiDou CNV3 message
//   CNV3,
//   /// CNVX special marker
//   CNVX,
// };

}  // namespace navp
