#include "space_time/time.hpp"

namespace nav {

namespace details {
std::tuple<long, long> convert_seconds(f64 seconds) {
  long sec = static_cast<long>(std::floor(seconds));
  long nanos = static_cast<long>((seconds - sec) * 1e9);
  return std::make_tuple(sec, nanos);
}

f64 convert_seconds(long second, long nanos) {
  return static_cast<f64>(second) + static_cast<f64>(nanos / 1e9);
}

bool is_leap_year(i32 year) { return (year % 400 == 0) || (year % 4 == 0 && year % 100 != 0); }
}  // namespace details

}  // namespace nav
