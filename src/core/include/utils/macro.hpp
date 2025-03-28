#pragma once

// version
#define NAVP_VERSION "0.1.0"

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

#define NAV_FORMAT std::format
#define NAV_PRINT std::print
#define NAV_PRINTLN std::println

// some alias
#define NAV_NODISCARD_UNUNSED [[__nodiscard__("unused result")]]
#define NAV_NODISCARD_ERROR_HANDLE [[__nodiscard__("the returning may contain an exception to be handled")]]
#define NAV_NODISCARD_WITHMSG(msg) [[__nodiscard__(msg)]]
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

// export
#ifdef NAVP_LIBRARY
#ifdef _WIN32
#define NAVP_EXPORT __declspec(dllexport)
#else
#define NAVP_EXPORT __attribute__((visibility("default")))
#endif
#else
#ifdef _WIN32
#define NAVP_EXPORT __declspec(dllimport)
#else
#define NAVP_EXPORT
#endif
#endif

// debug
#ifdef GNSS_DEBUG
#define ON_GNSS_DEBUG(code) code
#else
#define ON_GNSS_DEBUG(code)
#endif
