#pragma once

#include <format>
#include <sstream>
#include <string_view>

namespace navp::utils {

template <typename Char>
struct basic_ostream_formatter : std::formatter<std::basic_string_view<Char>, Char> {
  template <typename T, typename OutputIt>
  auto format(const T& value, std::basic_format_context<OutputIt, Char>& ctx) const -> OutputIt {
    std::basic_stringstream<Char> ss;
    ss << value;
    return std::formatter<std::basic_string_view<Char>, Char>::format(ss.view(), ctx);
  }
};

using ostream_formatter = basic_ostream_formatter<char>;

}  // namespace navp::utils