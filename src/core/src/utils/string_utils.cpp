#include "utils/string_utils.hpp"

namespace navp::utils {

void trim_left(std::string_view& str_view) noexcept {
  const auto str_begin = str_view.find_first_not_of(" \t\n\r\f\v");
  if (str_begin != std::string_view::npos) {
    str_view.remove_prefix(str_begin);
  } else {
    str_view.remove_prefix(str_view.size());
  }
}

void trim_right(std::string_view& str_view) noexcept {
  const auto str_end = str_view.find_last_not_of(" \t\n\r\f\v");
  if (str_end != std::string_view::npos) {
    str_view.remove_suffix(str_view.size() - str_end - 1);
  } else {
    str_view.remove_suffix(str_view.size());
  }
}

void trim(std::string_view& str_view) noexcept {
  trim_left(str_view);
  trim_right(str_view);
}

}  // namespace navp::utils