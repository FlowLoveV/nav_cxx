#include "io/rinex/rinex_reader.hpp"
#include "io/rinex/rinex_record.hpp"
#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

using navp::sensors::gnss::GnssNavRecord;
using navp::sensors::gnss::GnssObsRecord;
using navp::sensors::gnss::ObsList;

namespace navp::io::rinex {

RinexRecord::RinexRecord() : station_(std::make_unique<RinexStation>()), nav_(std::make_unique<Navigation>()) {}

RinexRecord::~RinexRecord() = default;

RinexStream::~RinexStream() = default;

void RinexStream::decode_record(Record& record) {
  // decode header
  if (tellg() == 0) {
    readRnxH(*this, version_, type_, sys_, tsys_, sys_code_types_, *nav_, *station_, glo_fcn_, glo_cpbias_);
    switch (type_) {
      case 'O': {
        if (auto gnss_obs = dynamic_cast<GnssObsRecord*>(&record)) {
          memcpy(glo_fcn_, gnss_obs->glo_fcn_, 28);
          memcpy(glo_cpbias_, gnss_obs->glo_cpbias_, 32);
        }
        break;
      }
      case 'N':
        [[fallthrough]];
      case 'G':
        [[fallthrough]];
      case 'H':
        [[fallthrough]];
      case 'J':
        [[fallthrough]];
      case 'L':
        [[fallthrough]];
      case 'C': {
        if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
          gnss_nav->nav->ionMap = std::move(nav_->ionMap);
          gnss_nav->nav->stoMap = std::move(nav_->stoMap);
        }
      }
    }
  }
  if (type_ == ' ') {
    nav_warn("RinexStream receive an empty stream!");
  }
  // decode body
  i32 stat = 0;
  switch (type_) {
    case 'O': {
      if (auto gnss_obs = dynamic_cast<GnssObsRecord*>(&record); gnss_obs) {
        ObsList obs_list;
        stat = readRnxObs(*this, version_, tsys_, sys_code_types_, obs_list, *station_);
        gnss_obs->add_obs_list(std::move(obs_list));
        break;
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'N': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, version_, sys_, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'G': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, version_, ConstellationEnum::GLO, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'H': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, version_, ConstellationEnum::SBS, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'J': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, version_, ConstellationEnum::QZS, *gnss_nav->nav);  // extension
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'L': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, version_, ConstellationEnum::GAL, *gnss_nav->nav);  // extension
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'C': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxClk(*this, version_, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    default: {
      nav_error("Unknown rinex file type {}", type_);
      return;
    }
  }

  if (stat) {
    record_number++;
  }
}

void RinexStream::encode_record(Record& record) {
  // todo
  nav_error("Not implentted yet!");
  exit(-1);
}

}  // namespace navp::io::rinex
