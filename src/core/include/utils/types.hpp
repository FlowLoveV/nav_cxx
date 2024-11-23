#pragma once

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <stdfloat>

namespace navp {

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#ifdef __SIZEOF_INT128__
typedef unsigned __int128 u128;
typedef unsigned __int128 u_least64;
#else
typedef uint64_t u_least64;
#endif
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
#ifdef __SIZEOF_INT128__
typedef __int128 i128;
typedef __int128 i_least64;
#else
typedef int64_t i_least64;
#endif

// todo
#ifdef __STDCPP_FLOAT16_T__
typedef std::float16_t f16;
#else
typedef std::float_t f16;
#endif

#ifdef __STDCPP_FLOAT32_T__
typedef std::float32_t f32;
#else
typedef std::float_t f32;
#endif

#ifdef __STDCPP_FLOAT64_T__
typedef std::float64_t f64;
#else
typedef std::double_t f64;
#endif

#ifdef __STDCPP_FLOAT128_T__
typedef std::float128_t f128;
typedef std::float128_t f_least64;
#else
typedef long double f128;
typedef long double f_least64;
#endif

#ifdef __STDCPP_BFLOAT16_T__
typedef std::bfloat16_t bf16;
#endif

}  // namespace navp
