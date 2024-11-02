#include <ranges>

#include "ginan/constants.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

const Sig* GObs::find_code(ObsCodeEnum code) const noexcept {
  auto freq = ginan::code2Freq[Sat.constellation.id][code];
  if (sigsLists.contains(freq)) {
    for (const auto& sig : sigsLists.at(freq)) {
      if (sig.code == code) {
        return &sig;
      }
    }
  }
  return nullptr;
}

GObs::operator std::shared_ptr<GObs>() {
  auto pointer = std::make_shared<GObs>(*this);
  return pointer;
}

ObsList& ObsList::operator+=(const ObsList& right) {
  this->insert(this->end(), right.begin(), right.end());
  return *this;
}

GnssNavRecord::GnssNavRecord(Navigation&& _nav) noexcept : nav(std::make_shared<Navigation>(std::move(_nav))) {}

GnssNavRecord::GnssNavRecord(std::unique_ptr<Navigation>&& _nav_ptr) noexcept : nav(std::move(_nav_ptr)) {}

GnssObsRecord::GnssObsRecord(ObsList&& _obs) noexcept { add_obs_list(std::move(_obs)); }

GnssObsRecord::GnssObsRecord(std::unique_ptr<ObsList>&& _obs_ptr) noexcept { add_obs_list(std::move(*_obs_ptr)); }

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

EpochUtc GnssObsRecord::begin_time() const noexcept {
  if (auto gobs_itr = obs_map.begin(); gobs_itr != obs_map.end()) {
    return EpochUtc{(*gobs_itr).first};
  } else {
    nav_warn("error pointer_cast from Observation to GObs");
    cpptrace::generate_trace(1).print();
    return {};
  }
}

EpochUtc GnssObsRecord::end_time() const noexcept {
  if (auto gobs_itr = obs_map.begin(); gobs_itr != obs_map.end()) {
    return EpochUtc{(*gobs_itr).first};
  } else {
    nav_warn("error pointer_cast from Observation to GObs");
    cpptrace::generate_trace(1).print();
    return {};
  }
}

std::tuple<EpochUtc, EpochUtc> GnssObsRecord::period() const noexcept { return {begin_time(), end_time()}; }

std::vector<EpochUtc> GnssObsRecord::epoches() const noexcept {
  return obs_map | std::views::keys | std::ranges::to<std::vector>();
}

Option<std::vector<Sv>> GnssObsRecord::sv(EpochUtc time) const noexcept {
  if (obs_map.contains(time)) {
    return obs_map.at(time) | std::views::keys | std::ranges::to<std::vector>();
  } else {
    return None;
  }
}

// void GnssObsRecord::generate_obs_map() const noexcept {
//   for (const auto& obs_ptr : *obs_list) {
//     EpochUtc epoch(obs_ptr->time);
//     obs_map[epoch][obs_ptr->Sat] = obs_ptr;
//   }
// }

auto GnssObsRecord::query(EpochUtc time) const noexcept -> Option<std::map<Sv, std::shared_ptr<GObs>>> {
  if (obs_map.contains(time)) {
    return obs_map.at(time);
  } else {
    return None;
  }
}

auto GnssObsRecord::query(EpochUtc time, Sv sv) const noexcept -> Option<std::shared_ptr<GObs>> {
  if (obs_map.contains(time) && obs_map.at(time).contains(sv)) {
    return obs_map.at(time).at(sv);
  } else {
    return None;
  }
}

void GnssObsRecord::add_obs_list(ObsList&& obs_list) noexcept {
  for (const auto& obs_ptr : obs_list) {
    EpochUtc epoch(obs_ptr->time);
    obs_map[epoch][obs_ptr->Sat] = obs_ptr;
  }
}

void GnssObsRecord::add_record(GnssObsRecord&& record) noexcept {
  for (auto& [epoch, obs] : obs_map) {
    obs_map[epoch] = std::move(obs);
  }
}

GnssObsRecord::~GnssObsRecord() = default;

GnssNavRecord::GnssNavRecord() : nav(std::make_shared<Navigation>()) {}

GnssNavRecord::~GnssNavRecord() = default;

}  // namespace navp::sensors::gnss