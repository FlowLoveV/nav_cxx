#pragma once

/// export header
#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/gnss_handler.hpp"
#include "sensors/gnss/random.hpp"
#include "utils/null_mutex.hpp"

namespace navp::sensors::gnss {

template <typename Mutex = utils::null_mutex>
class NAVP_EXPORT Gnss : protected GnssStationHandler {
 public:
  Gnss(GnssStationHandler&& handler) noexcept : GnssStationHandler(std::move(handler)) {}
  Gnss(const Gnss&) noexcept = delete;
  Gnss(Gnss&&) noexcept = default;
  Gnss& operator=(const Gnss&) noexcept = delete;
  Gnss& operator=(Gnss&&) noexcept = default;
  virtual ~Gnss() override = default;

  virtual bool update_record() override {
    std::lock_guard<Mutex> lock(mutex_);
    return GnssStationHandler::update_record();
  }

  virtual void update_runtime_info() override {
    std::lock_guard<Mutex> lock(mutex_);
    GnssStationHandler::update_runtime_info();
  }

 private:
  Mutex mutex_;
};

using GnssSt = Gnss<utils::null_mutex>;
using GnssMt = Gnss<std::mutex>;

}  // namespace navp::sensors::gnss