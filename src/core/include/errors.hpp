#pragma once

#include <cstdint>
#include <source_location>
#include <string_view>

namespace navp {

struct ErrorId;
struct ErrorBase;
struct NavError;

struct ErrorId {
  enum _ErrorCode : uint16_t {
    DividZero = 0,
    InvSingularMatrix,
  };

  ErrorId() = default;
  ErrorId(_ErrorCode code) noexcept : code(code) {}
  ~ErrorId() = default;

  _ErrorCode code;
};

std::string_view ErrorMessage(ErrorId) noexcept;

class ErrorBase {
 private:
  std::string_view error_msg, additional_msg;

 public:
  ErrorBase() noexcept = default;
  ErrorBase(const std::string_view error_msg, const std::string_view addi_msg = "") noexcept
      : error_msg(error_msg), additional_msg(addi_msg) {}
  ~ErrorBase() noexcept = default;

  void warn(std::source_location location = std::source_location::current());

  void crash(std::source_location location = std::source_location::current());
};

class NavError : public ErrorBase {
 private:
 public:
  using ErrorBase::ErrorBase;
  NavError(ErrorId id, std::string_view addi_msg = "") : ErrorBase(ErrorMessage(id), addi_msg) {
    error_id = id;
  }
  ~NavError() = default;

  uint16_t warn(std::source_location location = std::source_location::current());

  uint16_t crash(std::source_location location = std::source_location::current());

 private:
  ErrorId error_id;
};

}  // namespace navp