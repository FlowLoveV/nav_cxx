#pragma once

// Considering that c++20,23 features is not yet stable,
// making macro substitutions for certain keywords is necessary

// macro ENMODULES, used to turn on c++ modules
#ifdef ENMODULES
#define NAV_IMPORT import
#define NAV_EXPORT export
#define NAV_MODULE module
#define STD_MODULE std
#define NAV_MODULE_NAME(name) navp##_##name
#define NAV_SUFFIX ixx
#else
#define NAV_IMPORT
#define NAV_EXPORT
#define NAV_MODULE
#define STD_MODULE
#define NAV_MODULE_NAME(name)
#define NAV_SUFFIX hpp
#endif

// macro FORMATLIB, decides which format library will be used
// 0 - fmt (https://github.com/fmtlib/fmt)
// 1 - c++20 format,c++23 print(which is not support on linux and macos)

#define NAV_FORMAT std::format
#define NAV_PRINT std::print
#define NAV_PRINTLN std::println

// some alias
#define NAV_NODISCARD [[nodiscard]]
#define NAV_NODISCARD_WITHMSG(msg) [[nodiscard(msg)]]
#define NAV_NOEXCEPT noexcept
#define NAV_UNUSED [[maybe_unused]]

// format num
#define FORMAT_NUM(outter_formatter, inner_formatter, num) \
  NAV_FORMAT(outter_formatter, NAV_FORMAT(inner_formatter, num))

// some attributes
#if defined(__GNUC__) || defined(__clang__)
#define ANNOTATE(attr) __attribute__((annotate(attr)))
#else
#define ANNOTATE(attr)
#endif
