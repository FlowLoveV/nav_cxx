#include <ranges>

#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

const Sig* GObs::find_code(ObsCodeEnum code) const noexcept {
  auto freq = Constants::code_to_freq_enum(Sat.constellation.id, code);
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
  if (auto gobs_itr = obs_map_.begin(); gobs_itr != obs_map_.end()) [[likely]] {
    return EpochUtc{(*gobs_itr).first};
  } else {
    nav_error("No observation in GnssObsRecord!");
    return {};
  }
}

EpochUtc GnssObsRecord::end_time() const noexcept {
  if (auto gobs_itr = obs_map_.rbegin(); gobs_itr != obs_map_.rend()) [[likely]] {
    return EpochUtc{(*gobs_itr).first};
  } else {
    nav_error("No observation in GnssObsRecord!");
    return {};
  }
}

std::tuple<EpochUtc, EpochUtc> GnssObsRecord::period() const noexcept { return {begin_time(), end_time()}; }

std::vector<EpochUtc> GnssObsRecord::epoches() const noexcept {
  return obs_map_ | std::views::keys | std::ranges::to<std::vector>();
}

std::vector<Sv> GnssObsRecord::sv_at(EpochUtc time) const noexcept {
  if (obs_map_.contains(time)) {
    return obs_map_.at(time) | std::views::keys | std::ranges::to<std::vector>();
  } else {
    return {};
  }
}

auto GnssObsRecord::query(EpochUtc time) const noexcept -> const ObsMap* {
  if (obs_map_.contains(time)) {
    return &obs_map_.at(time);
  } else {
    return nullptr;
  }
}

auto GnssObsRecord::query(EpochUtc time, Sv sv) const noexcept -> const std::shared_ptr<GObs> {
  if (obs_map_.contains(time) && obs_map_.at(time).contains(sv)) {
    return obs_map_.at(time).at(sv);
  } else {
    return nullptr;
  }
}

void GnssObsRecord::add_obs_list(ObsList&& obs_list) noexcept {
  for (const auto& obs_ptr : obs_list) {
    EpochUtc epoch(obs_ptr->time);
    obs_map_[epoch][obs_ptr->Sat] = obs_ptr;
  }
  erase();
}

GnssObsRecord& GnssObsRecord::merge_record(GnssObsRecord&& record) noexcept {
  obs_map_.merge(std::move(record.obs_map_));
  return *this;
}

void GnssObsRecord::erase() noexcept {
  if (storage_ < 0) return;
  while (obs_map_.size() > storage_) {
    obs_map_.erase(obs_map_.begin());
  }
}

i32 GnssObsRecord::storage() const noexcept { return storage_; }

GnssObsRecord& GnssObsRecord::set_storage(i32 storage) noexcept {
  storage_ = storage;
  return *this;
}

u32 GnssObsRecord::frequency() const noexcept { return frequceny_; }

GnssObsRecord& GnssObsRecord::set_frequcney(u32 frequency) noexcept {
  frequceny_ = frequency;
  return *this;
}

auto GnssObsRecord::operator[](i64 index) const -> const ObsMap* {
  if (obs_map_.empty()) {
    throw std::out_of_range("GnssObsRecord is empty");
  }

  if (index >= 0) {
    // Positive index
    auto it = obs_map_.begin();
    std::advance(it, index);
    if (it == obs_map_.end()) {
      throw std::out_of_range("Index out of range");
    }
    return &it->second;
  } else {
    // Negative index
    auto it = obs_map_.rbegin();
    std::advance(it, -index - 1);
    if (it == obs_map_.rend()) {
      throw std::out_of_range("Index out of range");
    }
    return &it->second;
  }
}

auto GnssObsRecord::latest() const -> const StorageType::value_type& { return *obs_map_.rend(); }

GnssObsRecord::~GnssObsRecord() = default;

GnssNavRecord::GnssNavRecord() : nav(std::make_shared<Navigation>()) {}

GnssNavRecord::~GnssNavRecord() = default;

}  // namespace navp::sensors::gnss