#pragma once

#include <variant>

namespace navp::utils {

template <typename... Args>
class Enum;

template <typename... Args>
class Enum : public std::variant<Args...> {
 public:
   
};

}  // namespace navp::utils
