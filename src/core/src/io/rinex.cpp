#include "io/rinex/rinex_reader.hpp"
#include "io/rinex/rinex_record.hpp"
#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

using navp::sensors::gnss::GnssNavRecord;
using navp::sensors::gnss::GnssObsRecord;
using navp::sensors::gnss::ObsList;

namespace navp::io::rinex {

RinexRecord::RinexRecord() : _station(std::make_shared<RinexStation>()), _nav(std::make_shared<Navigation>()) {}

RinexRecord::~RinexRecord() = default;

RinexStream::~RinexStream() = default;

void RinexStream::decode_record(Record& record) {
  // decode header
  if (tellg() == 0) {
    readRnxH(*this, _version, _type, _sys, _tsys,
             _sys_code_types, *_nav, *_station);
  }
  if (_type == ' ') {
    nav_warn("RinexStream receive an empty stream!");
  }
  // decode body
  i32 stat = 0;
  switch (_type) {
    case 'O': {
      if (auto gnss_obs = dynamic_cast<GnssObsRecord*>(&record); gnss_obs) {
        ObsList obs_list;
        stat = readRnxObs(*this, _version, _tsys, _sys_code_types, obs_list,
                          *_station);
        gnss_obs->add_obs_list(std::move(obs_list));
        break;
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'N': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, _version, _sys, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'G': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, _version, ConstellationEnum::GLO, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'H': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, _version, ConstellationEnum::SBS, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'J': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, _version, ConstellationEnum::QZS, *gnss_nav->nav);  // extension
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'L': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxNav(*this, _version, ConstellationEnum::GAL, *gnss_nav->nav);  // extension
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
      return;
    }

    case 'C': {
      if (auto gnss_nav = dynamic_cast<GnssNavRecord*>(&record)) {
        stat = readRnxClk(*this, _version, *gnss_nav->nav);
      } else {
        nav_warn("RinexStream decode unmatched record!");
      }
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
