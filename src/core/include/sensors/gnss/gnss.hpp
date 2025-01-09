#pragma once

/// export header
#include "sensors/gnss/gnss_handler.hpp"
#include "utils/null_mutex.hpp"

namespace navp::sensors::gnss {
using navp::EpochUtc;
class GnssHandler;
template <typename Mutex>
class Gnss;

class NAVP_EXPORT GnssHandler {
 public:
  virtual ~GnssHandler() = default;

  virtual auto station_info() const noexcept -> const GnssStationInfo* = 0;

  virtual auto runtime_info() const noexcept -> const GnssRuntimeInfo* = 0;

  virtual auto record() const noexcept -> const GnssRecord* = 0;

  virtual auto settings() const noexcept -> const GnssSettings* = 0;

  virtual auto logger() const noexcept -> const std::shared_ptr<spdlog::logger>& = 0;

  virtual bool update_record() = 0;

  virtual auto update_runtime_info() -> const GnssRuntimeInfo* = 0;

  virtual auto generate_rawobs_handler() const -> std::vector<GnssRawObsHandler> = 0;

  virtual auto generate_undiffobs_handler() const -> std::vector<UnDiffObsHandler> = 0;

  virtual auto generate_atmosphere_handler(Sv sv) const -> AtmosphereHandler = 0;

  virtual auto generate_random_handler(Sv sv) const -> GnssRandomHandler = 0;
};

template <typename Mutex = utils::null_mutex>
class NAVP_EXPORT Gnss : public GnssHandler, public GnssPayload {
 public:
  Gnss(GnssPayload&& handler) noexcept : GnssPayload(std::move(handler)) {}
  Gnss(const Gnss&) noexcept = delete;
  Gnss(Gnss&&) noexcept = default;
  Gnss& operator=(const Gnss&) noexcept = delete;
  Gnss& operator=(Gnss&&) noexcept = default;
  virtual ~Gnss() override = default;

  virtual auto station_info() const noexcept -> const GnssStationInfo* override { return station_info_.get(); }

  virtual auto runtime_info() const noexcept -> const GnssRuntimeInfo* override { return runtime_info_.get(); }

  virtual auto record() const noexcept -> const GnssRecord* override { return record_.get(); }

  virtual auto settings() const noexcept -> const GnssSettings* override { return settings_.get(); }

  virtual auto logger() const noexcept -> const std::shared_ptr<spdlog::logger>& override { return logger_; }

  virtual bool update_record() override {
    std::lock_guard<Mutex> lock(mutex_);
    if (GnssPayload::update_record()) {
      return true;
    }
    return false;
  }

  virtual auto update_runtime_info() -> const GnssRuntimeInfo* override {
    std::lock_guard<Mutex> lock(mutex_);
    GnssPayload::update_runtime_info();
    return runtime_info_.get();
  }

  virtual auto generate_rawobs_handler() const -> std::vector<GnssRawObsHandler> override {
    std::lock_guard<Mutex> lock(mutex_);
    return GnssPayload::generate_rawobs_handler();
  }

  virtual auto generate_undiffobs_handler() const -> std::vector<UnDiffObsHandler> override {
    std::lock_guard<Mutex> lock(mutex_);
    return GnssPayload::generate_undiffobs_handler();
  }

  virtual auto generate_atmosphere_handler(Sv sv) const -> AtmosphereHandler override {
    std::lock_guard<Mutex> lock(mutex_);
    return GnssPayload::generate_atmosphere_handler(sv);
  }

  virtual auto generate_random_handler(Sv sv) const -> GnssRandomHandler override {
    std::lock_guard<Mutex> lock(mutex_);
    return GnssPayload::generate_random_handler(sv);
  }

 private:
  mutable Mutex mutex_;
};

using GnssSt = Gnss<utils::null_mutex>;
using GnssMt = Gnss<std::mutex>;

}  // namespace navp::sensors::gnss