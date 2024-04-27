#pragma once

// Considering that c++20,23 features is not yet stable,
// making macro substitutions for certain keywords is necessary

#ifdef ENMODULES
#define NAV_IMPORT import
#define NAV_EXPORT export
#define NAV_MODULE module
#define STD_MODULE std
#define NAV_MODULE_NAME(name) nav##.##name
#define NAV_SUFFIX ixx
#else
#define NAV_IMPORT
#define NAV_EXPORT
#define NAV_MODULE
#define STD_MODULE
#define NAV_MODULE_NAME(name)
#define NAV_SUFFIX hpp
#endif

// some alias
#define NAV_NODISCARD [[nodiscard]]
#define NAV_NODISCARD_WITHMSG(msg) [[nodiscard(msg)]]
#define NAV_NOEXCEPT noexcept
#define NAV_UNUSED [[maybe_unused]]
