#include "utils/time.hpp"

namespace navp {

namespace details {
std::tuple<long, long> convert_seconds(f64 seconds) {
  long sec = static_cast<long>(std::floor(seconds));
  long nanos = static_cast<long>((seconds - sec) * 1e9);
  return std::make_tuple(sec, nanos);
}

f64 convert_seconds(long second, long nanos) { return static_cast<f64>(second) + static_cast<f64>(nanos / 1e9); }

bool is_leap_year(i32 year) { return (year % 400 == 0) || (year % 4 == 0 && year % 100 != 0); }
}  // namespace details

Epoch<GPST>::Epoch(GTime gtime) noexcept {
  auto epoch = Epoch::from_seconds(gtime.bigTime);
  *this = epoch;
}

Epoch<GPST>::operator GTime() const noexcept {
  long double t = *this;
  GTime res;
  res.bigTime = t;
  return res;
}

Epoch<UTC>::Epoch(GTime gtime) noexcept {
  auto epoch_gpst = Epoch<GPST>::from_seconds(gtime.bigTime);
  *this = epoch_cast<UTC>(epoch_gpst);
}

Epoch<UTC>::operator GTime() const noexcept {
  auto epoch_gpst = epoch_cast<GPST>(*this);
  return epoch_gpst;
}

Epoch<BDT>::Epoch(GTime gtime) noexcept {
  auto epoch_gpst = Epoch<GPST>::from_seconds(gtime.bigTime);
  *this = epoch_cast<BDT>(epoch_gpst);
}

Epoch<BDT>::operator GTime() const noexcept {
  auto epoch_gpst = epoch_cast<GPST>(*this);
  return epoch_gpst;
}

}  // namespace navp
