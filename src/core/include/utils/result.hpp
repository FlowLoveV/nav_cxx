#pragma once

#include <cpptrace/cpptrace.hpp>
#include <optional>
#include <string>
#include <variant>

#include "logger.hpp"
namespace navp {

struct Error {
  unsigned code;
  std::string message;

  template <typename T>
    requires std::is_base_of_v<Error, T>
  bool operator==(const T& err) noexcept {
    return code == err.code;
  }

  void crash() {
    nav_error("{}", message);
    cpptrace::generate_trace(1).print();
    exit(-1);
  }
};

template <typename T, typename E = Error>
class Result : public std::variant<T, E> {
 public:
  using std::variant<T, E>::variant;
  // is_ok,is_err
  constexpr inline bool is_ok() noexcept { return this->index() == 0; }
  constexpr inline bool is_err() noexcept { return this->index() == 1; }
  // ok,err
  // constexpr std::optional<T>

  // unwrap
  constexpr T& unwrap() & {
    if (is_ok()) [[likely]] {
      return _m_get_ok_value();
    } else {
      // call destruct fucntion to terminate
      ~_m_get_err_value();
    }
  }
  constexpr const T& unwrap() const& {
    if (is_ok()) [[likely]] {
      return std::get<T>(*this);
    } else {
      // call destruct fucntion to terminate
      auto err = std::get<E>(*this);
      ~err();
    }
  }
  constexpr const T&& unwrap() const&& {
    if (is_ok()) [[likely]] {
      return std::move(std::get<T>(*this));
    } else {
      // call destruct fucntion to terminate
      auto err = std::get<E>(*this);
      ~err();
    }
  }
  constexpr T&& unwrap() && {
    if (is_ok()) [[likely]] {
      return std::move(std::get<T>(*this));
    } else {
      // call destruct fucntion to terminate
      auto err = std::get<E>(*this);
      ~err();
    }
  }

  // unwrap_or
  template <typename U>
  constexpr T unwrap_or(U&& _default) const& noexcept(
      std::__and_v<std::is_nothrow_copy_constructible<T>, std::is_nothrow_convertible<U, T>>) {
    static_assert(std::is_copy_constructible_v<T>);
    static_assert(std::is_convertible_v<U, T>);
    return is_ok() ? _m_get_ok_value() : static_cast<T>(std::forward<U>(_default));
  }
  template <typename U>
  constexpr T unwrap_or(U&& _default) && noexcept(
      std::__and_v<std::is_nothrow_move_constructible<T>, std::is_nothrow_convertible<U, T>>) {
    static_assert(std::is_copy_constructible_v<T>);
    static_assert(std::is_convertible_v<U, T>);
    return is_ok() ? std::move(_m_get_ok_value()) : static_cast<T>(std::forward<U>(_default));
  }

  // unwarp_or_else

  // is_ok

  // is_ok_and

  // is_err

  // is_err_and

  // and_then

 protected:
  constexpr inline auto _m_get_ok_value() { return std::get<T>(*this); }
  constexpr inline auto _m_get_err_value() { return std::get<E>(*this); }
};

// struct ErrorCode {
//   template <typename T>
//     requires std::is_base_of_v<ErrorCode, T>
//   bool operator==(const T& _code) noexcept {
//     if constexpr (std::is_same_v<std::decay_t<decltype(*this)>, std::decay_t<T>>) {
//       return code == _code.code;
//     } else {
//       return false;
//     }
//   }

//   ErrorCode operator=(unsigned _code) noexcept { return ErrorCode{.code = _code}; }
//   operator unsigned() noexcept { return code; }

//   unsigned code;
// };

// struct Error {
//   ErrorCode code;
//   std::string message;

//   template <typename T>
//     requires std::is_base_of_v<Error, T>
//   bool operator==(const T& err) noexcept {
//     return code == err.code;
//   }

//   void crash() {
//     nav_error("{}", message);
//     cpptrace::generate_trace(1).print();
//     exit(-1);
//   }
// };

// template <typename T, typename E = Error>
// struct Result : public std::expected<T, E> {};

// template <typename... Args>
// constexpr inline static Error make_error(ErrorCode code, const std::string& format_str, Args&&... args) {
//   return Error{.code = code, .message = std::vformat(format_str,
//   std::make_format_args(std::forward<Args>(args)...))};
// }

}  // namespace navp