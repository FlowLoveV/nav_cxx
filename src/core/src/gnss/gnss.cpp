#include <ranges>

#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

#define NSYSGPS 1
#define NSATGPS 32  ///< potential number of GPS satellites, PRN goes from 1 to this number
#define NSATGLO 27  ///< potential number of GLONASS satellites, PRN goes from 1 to this number
#define NSATGAL 36  ///< potential number of Galileo satellites, PRN goes from 1 to this number
#define NSATQZS 7   ///< potential number of QZSS satellites, PRN goes from 1 to this number
#define NSATLEO 78  ///< potential number of LEO satellites, PRN goes from 1 to this number
#define NSATBDS 62  ///< potential number of Beidou satellites, PRN goes from 1 to this number
#define NSATSBS 39  ///< potential number of SBAS satellites, PRN goes from 1 to this number

const Sig* GObs::find_code(ObsCodeEnum code) const noexcept {
  auto freq = Constants::code_to_freq_enum(sv.system(), code);
  if (sigs_list.contains(freq)) {
    for (const auto& sig : sigs_list.at(freq)) {
      if (sig.code == code) {
        return &sig;
      }
    }
  }
  return nullptr;
}

u8 GObs::frequency_count() const noexcept { return sigs_list.size(); }

u8 GObs::code_count() const noexcept {
  u8 count = 0;
  std::ranges::for_each(sigs_list | std::views::values, [&](const std::list<Sig>& sig) { count += sig.size(); });
  return count;
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
  if (cons == ConstellationEnum::GPS) {
    sats.resize(NSATGPS);
    for (u8 prn = 1; prn <= NSATGPS; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::GPS};
    }
  }
  if (cons == ConstellationEnum::GLO) {
    sats.resize(NSATGLO);
    for (u8 prn = 1; prn <= NSATGLO; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::GLO};
    }
  }
  if (cons == ConstellationEnum::GAL) {
    sats.resize(NSATGAL);
    for (u8 prn = 1; prn <= NSATGAL; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::GAL};
    }
  }
  if (cons == ConstellationEnum::BDS) {
    sats.resize(NSATBDS);
    for (u8 prn = 1; prn <= NSATBDS; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::BDS};
    }
  }
  if (cons == ConstellationEnum::QZS) {
    sats.resize(NSATQZS);
    for (u8 prn = 1; prn <= NSATQZS; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::QZS};
    }
  }
  if (cons == ConstellationEnum::SBS) {
    sats.resize(NSATSBS);
    for (u8 prn = 1; prn <= NSATSBS; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::SBS};
    }
  }
  if (cons == ConstellationEnum::LEO) {
    sats.resize(NSATLEO);
    for (u8 prn = 1; prn <= NSATLEO; prn++) {
      sats[prn - 1] = Sv{prn, ConstellationEnum::LEO};
    }
  }
  return sats;
};

EpochUtc GnssObsRecord::begin_time() const noexcept { return obs_map_.begin()->first; }

EpochUtc GnssObsRecord::end_time() const noexcept { return obs_map_.rbegin()->first; }

std::tuple<EpochUtc, EpochUtc> GnssObsRecord::period() const noexcept { return {begin_time(), end_time()}; }

std::vector<EpochUtc> GnssObsRecord::epoches() const noexcept {
  return obs_map_ | std::views::keys | std::ranges::to<std::vector>();
}

std::vector<Sv> GnssObsRecord::sv_at(EpochUtc time) const noexcept {
  return obs_map_.at(time) | std::views::keys | std::ranges::to<std::vector>();
}

auto GnssObsRecord::at(EpochUtc time) const noexcept -> const ObsMap* { return &obs_map_.at(time); }

auto GnssObsRecord::at(EpochUtc time, Sv sv) const noexcept -> const GObs* { return obs_map_.at(time).at(sv).get(); }

auto GnssObsRecord::contains(EpochUtc time) const noexcept -> bool { return obs_map_.contains(time); }

void GnssObsRecord::add_obs_list(ObsList&& obs_list) noexcept {
  for (const auto& obs_ptr : obs_list) {
    EpochUtc epoch(obs_ptr->time);
    obs_map_[epoch][obs_ptr->sv] = obs_ptr;
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

GnssObsRecord& GnssObsRecord::set_frequency(u32 frequency) noexcept {
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

auto GnssObsRecord::latest() const -> const StorageType::value_type& { return *obs_map_.rbegin(); }

GnssObsRecord::~GnssObsRecord() = default;

GnssNavRecord::GnssNavRecord() : nav(std::make_shared<Navigation>()) {}

GnssNavRecord::~GnssNavRecord() = default;

#undef NSYSGPS
#undef NSATGPS
#undef NSATGLO
#undef NSATGAL
#undef NSATQZS
#undef NSATLEO
#undef NSATBDS
#undef NSATSBS

}  // namespace navp::sensors::gnss