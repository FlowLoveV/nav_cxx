#include <ranges>

#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "sensors/gnss/sv.hpp"

namespace navp::sensors::gnss {

RecordGnssNav::RecordGnssNav(Navigation&& _nav) noexcept : nav(std::make_shared<Navigation>(std::move(_nav))) {}

RecordGnssNav::RecordGnssNav(std::unique_ptr<Navigation>&& _nav_ptr) noexcept : nav(std::move(_nav_ptr)) {}

RecordGnssObs::RecordGnssObs(ObsList&& _obs) noexcept : obs_list(std::make_unique<ObsList>(std::move(_obs))) {
  generate_obs_map();
}

RecordGnssObs::RecordGnssObs(std::unique_ptr<ObsList>&& _obs_ptr) noexcept : obs_list(std::move(_obs_ptr)) {
  generate_obs_map();
}

std::vector<Sv> get_sv_sats(ConstellationEnum cons) {
  std::vector<Sv> sats;
  /* output satellite PRN*/
  if (cons == ConstellationEnum::GPS)
    for (u8 prn = 1; prn <= NSATGPS; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::GPS});
    }
  if (cons == ConstellationEnum::GLO)
    for (u8 prn = 1; prn <= NSATGLO; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::GLO});
    }
  if (cons == ConstellationEnum::GAL)
    for (u8 prn = 1; prn <= NSATGAL; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::GAL});
    }
  if (cons == ConstellationEnum::BDS)
    for (u8 prn = 1; prn <= NSATBDS; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::BDS});
    }
  if (cons == ConstellationEnum::QZS)
    for (u8 prn = 1; prn <= NSATQZS; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::QZS});
    }
  if (cons == ConstellationEnum::SBS)
    for (u8 prn = 1; prn <= NSATSBS; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::SBS});
    }
  if (cons == ConstellationEnum::LEO)
    for (u8 prn = 1; prn <= NSATLEO; prn++) {
      sats.push_back(Sv{prn, ConstellationEnum::LEO});
    }

  return sats;
};

Epoch<UTC> RecordGnssObs::begin_time() const noexcept {
  if (auto gobs_ptr = std::dynamic_pointer_cast<GObs>(obs_list->front()); gobs_ptr) {
    return Epoch<UTC>{gobs_ptr->time};
  } else {
    nav_warn("error pointer_cast from Observation to GObs");
    cpptrace::generate_trace(1).print();
    return {};
  }
}

Epoch<UTC> RecordGnssObs::end_time() const noexcept {
  if (auto gobs_ptr = std::dynamic_pointer_cast<GObs>(obs_list->back()); gobs_ptr) {
    return Epoch<UTC>{gobs_ptr->time};
  } else {
    nav_warn("error pointer_cast from Observation to GObs");
    cpptrace::generate_trace(1).print();
    return {};
  }
}

std::tuple<Epoch<UTC>, Epoch<UTC>> RecordGnssObs::period() const noexcept { return {begin_time(), end_time()}; }

std::vector<Epoch<UTC>> RecordGnssObs::epoches() const noexcept {
  return obs_map | std::views::keys | std::ranges::to<std::vector>();
}

Option<std::vector<Sv>> RecordGnssObs::sv(Epoch<UTC> time) const noexcept {
  if (obs_map.contains(time)) {
    return obs_map[time] | std::views::keys | std::ranges::to<std::vector>();
  } else {
    return None;
  }
}

void RecordGnssObs::generate_obs_map() const noexcept {
  for (const auto& obs_ptr : *obs_list) {
    if (auto gobs_ptr = std::dynamic_pointer_cast<GObs>(obs_ptr); gobs_ptr) {
      Epoch<UTC> epoch(gobs_ptr->time);
      obs_map[epoch][gobs_ptr->Sat] = gobs_ptr->sigsLists;
    }
  }
}

auto RecordGnssObs::query(Epoch<UTC> time) const noexcept
    -> Option<std::map<Sv, std::map<FreTypeEnum, std::list<Sig>>>> {
  if (obs_map.contains(time)) {
    return obs_map[time];
  } else {
    return None;
  }
}

auto RecordGnssObs::query(Epoch<UTC> time, Sv sv) const noexcept -> Option<std::map<FreTypeEnum, std::list<Sig>>> {
  if (obs_map.contains(time) && obs_map[time].contains(sv)) {
    return obs_map[time][sv];
  } else {
    return None;
  }
}

}  // namespace navp::sensors::gnss