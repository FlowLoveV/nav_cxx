#include <boost/algorithm/string.hpp>
#include <ranges>

#include "sensors/gnss/carrier.hpp"
#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "sensors/gnss/sv.hpp"

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
    throw GnssObsRecordError("GnssObsRecord is empty");
  }

  if (index >= 0) {
    // Positive index
    auto it = obs_map_.begin();
    std::advance(it, index);
    if (it == obs_map_.end()) {
      throw GnssObsRecordError("Index out of range");
    }
    return &it->second;
  } else {
    // Negative index
    auto it = obs_map_.rbegin();
    std::advance(it, index);
    if (it == obs_map_.rend()) {
      throw GnssObsRecordError("Index out of range");
    }
    return &it->second;
  }
}

auto GnssObsRecord::latest() const -> const StorageType::value_type& { return *obs_map_.rbegin(); }

GnssObsRecord::~GnssObsRecord() = default;

GnssNavRecord::GnssNavRecord() : nav(std::make_shared<Navigation>()) {}

GnssNavRecord::~GnssNavRecord() = default;

const CodeMap& GnssObsRecord::code_map() const noexcept { return code_map_; }

GnssObsRecord::GnssObsRecord(std::shared_ptr<spdlog::logger> logger) : logger_(logger) {}

void GObs::check_vaild() noexcept {
  std::ranges::for_each(sigs_list | std::views::values | std::views::join, [](Sig& sig) {
    if (sig.pseudorange == 0.0 || sig.carrier == 0.0 || sig.doppler == 0.0 || sig.snr == 0.0) {
      sig.invalid = true;
    }
  });
}

Result<Carrier, GnssParseCarrierError> Carrier::from_str(const char* str) {
  std::string s(str);
  boost::algorithm::to_upper(s);
  boost::algorithm::trim(s);
  /*
   * GPS, Galieo
   */
  auto result = magic_enum::enum_cast<CarrierEnum>(str);
  if (result.has_value()) {
    return Ok(Carrier(result.value()));
  }
  return Err(GnssParseCarrierError(std::format("can't parse unknown Carrier \'{}\'", str)));
}

Result<Constellation, GnssParseConstellationError> Constellation::form_str(const char* str) {
  std::string s(str);
  boost::algorithm::trim(s);
  boost::algorithm::to_lower(s);
  ConstellationEnum result;
  if (s == "g" || s == "gps")
    result = ConstellationEnum::GPS;
  else if (s == "c" || s == "bds")
    result = ConstellationEnum::BDS;
  else if (s == "e" || s == "gal")
    result = ConstellationEnum::GAL;
  else if (s == "r" || s == "glo")
    result = ConstellationEnum::GLO;
  else if (s == "j" || s == "qzss")
    result = ConstellationEnum::QZS;
  else if (s == "i" || s == "irnss")
    result = ConstellationEnum::IRN;
  else if (s == "s" || s == "sbas")
    result = ConstellationEnum::SBS;
  else if (s == "m" || s == "mixed")
    result = ConstellationEnum::Mixed;
  else if (s == "ausnz")
    result = ConstellationEnum::AusNZ;
  else if (s == "egnos")
    result = ConstellationEnum::EGNOS;
  else if (s == "waas")
    result = ConstellationEnum::WAAS;
  else if (s == "kass")
    result = ConstellationEnum::KASS;
  else if (s == "gagan")
    result = ConstellationEnum::GAGAN;
  else if (s == "asbas")
    result = ConstellationEnum::ASBAS;
  else if (s == "nsas")
    result = ConstellationEnum::NSAS;
  else if (s == "asal")
    result = ConstellationEnum::ASAL;
  else if (s == "msas")
    result = ConstellationEnum::MSAS;
  else if (s == "span")
    result = ConstellationEnum::SPAN;
  else if (s == "gbas")
    result = ConstellationEnum::GBAS;
  else if (s == "sdcm")
    result = ConstellationEnum::SDCM;
  else if (s == "bdsbas")
    result = ConstellationEnum::BDSBAS;
  else if (s.find("gps") != std::string::npos)
    result = ConstellationEnum::GPS;
  else if (s.find("glonass") != std::string::npos)
    result = ConstellationEnum::GLO;
  else if (s.find("beidou") != std::string::npos)
    result = ConstellationEnum::BDS;
  else if (s.find("galileo") != std::string::npos)
    result = ConstellationEnum::GAL;
  else if (s.find("qzss") != std::string::npos)
    result = ConstellationEnum::QZS;
  else if (s.find("sbas") != std::string::npos || s.find("geo") != std::string::npos)
    result = ConstellationEnum::SBS;
  else if (s.find("irnss") != std::string::npos || s.find("navic") != std::string::npos)
    result = ConstellationEnum::IRN;
  else if (s.find("mix") != std::string::npos)
    result = ConstellationEnum::Mixed;
  else {
    return Err(GnssParseConstellationError(std::format("can't parse unknwon Constellation \'{}\'", str)));
  }

  return Ok(Constellation(result));
}

bool Constellation::is_sbas() const {
  return id == ConstellationEnum::WAAS || id == ConstellationEnum::EGNOS || id == ConstellationEnum::MSAS ||
         id == ConstellationEnum::GAGAN || id == ConstellationEnum::BDSBAS || id == ConstellationEnum::KASS ||
         id == ConstellationEnum::SDCM || id == ConstellationEnum::ASBAS || id == ConstellationEnum::SPAN ||
         id == ConstellationEnum::SBS || id == ConstellationEnum::AusNZ || id == ConstellationEnum::GBAS ||
         id == ConstellationEnum::NSAS || id == ConstellationEnum::ASAL;
}

bool Constellation::is_mixed() const { return id == ConstellationEnum::Mixed; }

std::string_view Constellation::name() const { return magic_enum::enum_name(id); }

bool Constellation::operator==(const Constellation& rhs) const { return this->id == rhs.id; }

bool Constellation::operator!=(const Constellation& rhs) const { return this->id != rhs.id; }

std::strong_ordering Constellation::operator<=>(const Constellation& rhs) const { return id <=> rhs.id; }

Result<Sv, GnssRuntimeError> Sv::from_str(const char* str) {
  std::string_view view(&str[1]);
  char first[] = {str[0], '\0'};
  auto constellation = Constellation::form_str(first);
  if (constellation.is_err()) {
    return constellation.unwrap_err();
  }
  u8 prn;
  if (std::all_of(view.begin(), view.end(), [](char c) { return std::isspace(c); })) {
    prn = 0;
  } else {
    auto [ptr, ec] = std::from_chars(view.data(), view.data() + view.size(), prn);
    if (ec != std::errc()) {
      return Err(GnssParseSvError(std::format("can't parse unknown Sv \'{}\'", str)));
    }
  }
  return Ok(Sv{prn, constellation.unwrap_unchecked()});
};

bool Sv::operator!=(const Sv& rhs) const noexcept { return !(*this == rhs); }

bool Sv::operator==(const Sv& rhs) const noexcept {
  // When prn is 0, Sv degenerates into a Constellation identifier, which is used to determine
  // whether it is the same Constellation.
  if (this->prn == 0 || rhs.prn == 0) {
    return this->constellation == rhs.constellation;
  }
  return prn == rhs.prn && constellation == rhs.constellation;
}

std::strong_ordering Sv::operator<=>(const Sv& rhs) const {
  if (rhs.constellation != constellation) {
    return rhs.system() <=> system();
  }
  return prn <=> rhs.prn;
}

Sv::operator bool() const noexcept { return !(system() == ConstellationEnum::NONE || prn == 0); }

ConstellationEnum Sv::system() const noexcept { return constellation.id; }

ConstellationEnum& Sv::system() noexcept { return constellation.id; }

#undef NSYSGPS
#undef NSATGPS
#undef NSATGLO
#undef NSATGAL
#undef NSATQZS
#undef NSATLEO
#undef NSATBDS
#undef NSATSBS

}  // namespace navp::sensors::gnss