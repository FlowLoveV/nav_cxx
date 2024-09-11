#pragma once

#include <cpptrace/cpptrace.hpp>
#include <expected>
#include <string>

#include "logger.hpp"
namespace navp {

struct ErrorCode {
  template <typename T>
    requires std::is_base_of_v<ErrorCode, T>
  bool operator==(const T& _code) noexcept {
    if constexpr (std::is_same_v<std::decay_t<decltype(*this)>, std::decay_t<T>>) {
      return code == _code.code;
    } else {
      return false;
    }
  }

  ErrorCode operator=(unsigned _code) noexcept { return ErrorCode{.code = _code}; }
  operator unsigned() noexcept { return code; }

  unsigned code;
};

struct Error {
  ErrorCode code;
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
struct Result : public std::expected<T, E> {
      
};

template <typename... Args>
constexpr inline static Error make_error(ErrorCode code, const std::string& format_str, Args&&... args) {
  return Error{.code = code, .message = std::vformat(format_str, std::make_format_args(std::forward<Args>(args)...))};
}

}  // namespace navp