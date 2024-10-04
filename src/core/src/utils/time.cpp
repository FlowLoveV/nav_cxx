#include "utils/time.hpp"

namespace navp {

using utils::GTime;

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
  f128 t = *this;
  GTime res;
  res.bigTime = t / 1e9;
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

Epoch<UTC>::Epoch(const ReflectionType& _impl) {
  auto tm = _impl.time.get().tm();
  auto utc = Epoch<UTC>::from_utc_date(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  this->tp = utc.tp;
}

Epoch<GPST>::Epoch(const ReflectionType& _impl) {
  auto tm = _impl.time.get().tm();
  auto utc = Epoch<UTC>::from_utc_date(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  auto gpst = epoch_cast<GPST>(utc);
  this->tp = gpst.tp;
}

Epoch<BDT>::Epoch(const ReflectionType& _impl) {
  auto tm = _impl.time.get().tm();
  auto utc = Epoch<UTC>::from_utc_date(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  auto bdt = epoch_cast<BDT>(utc);
  this->tp = bdt.tp;
}

}  // namespace navp
