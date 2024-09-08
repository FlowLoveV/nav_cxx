#pragma once

#include <concepts>
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace navp {

template <std::floating_point T>
T to_radians(T val) {
  return val / 180.0 * std::numbers::pi;
}

template <std::floating_point T>
T to_degress(T val) {
  return val / std::numbers::pi * 180.0;
}

template <typename T>
bool checkArraysEqual(T* lhs, T* rhs, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (lhs[i] != rhs[i]) {
      return false;
    }
  }
  return true;
}

template <typename T>
T Square(T t) {
  return t * t;
}

// get cube of T value
template <typename T>
T Cube(T t) {
  return t * t * t;
}

constexpr double ct_sqrt(double x, double current, double prev) {
  return (current == prev) ? current : ct_sqrt(x, 0.5 * (current + x / current), current);
}

// concepts
template <typename T>
concept array_type = std::is_array_v<T>;
template <typename T>
concept pointer_type = std::is_pointer_v<T>;

}  // namespace navp
