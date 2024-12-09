#pragma once

#include <stdexcept>

#include "utils/macro.hpp"

namespace navp {

class NavException;
class NavRuntimeError;

class NAVP_EXPORT NavException : public std::exception {
 public:
  using std::exception::exception;
  using std::exception::operator=;
};

class NAVP_EXPORT NavRuntimeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
  using std::runtime_error::operator=;
};

#define REGISTER_NAV_EXCEPTION_CHILD(name)       \
  class NAVP_EXPORT name : public NavException { \
   public:                                       \
    using NavException::NavException;            \
    using NavException::operator=;               \
  };

#define IMPLMENT_NAV_EXCEPTION_CHILD(exception_name, base_name, message)   \
  class NAVP_EXPORT exception_name : public base_name {                    \
   public:                                                                 \
    using base_name::base_name;                                            \
    using base_name::operator=;                                            \
    virtual const char* what() const noexcept override { return message; } \
  };

#define REGISTER_NAV_RUNTIME_ERROR_CHILD(runtime_error_name, base_name) \
  class NAVP_EXPORT runtime_error_name : public base_name {             \
   public:                                                              \
    using base_name::base_name;                                         \
    using base_name::operator=;                                         \
  };

}  // namespace navp