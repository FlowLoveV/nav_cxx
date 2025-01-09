#pragma once

#include <string_view>

namespace navp::utils {

void trim_left(std::string_view& str_view) noexcept;

void trim_right(std::string_view& str_view) noexcept;

void trim(std::string_view& str_view) noexcept;

}  // namespace navp::utils