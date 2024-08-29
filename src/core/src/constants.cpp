#include "constants/carrier_table.hpp"

namespace nav::constants {

const boost::bimap<std::string, CarrierEnum> CARRIER_TABLE = []() {
  boost::bimap<std::string, CarrierEnum> bimap;
  bimap.insert({"L1", CarrierEnum::L1});
  bimap.insert({"L2", CarrierEnum::L2});
  bimap.insert({"L5", CarrierEnum::L5});
  bimap.insert({"L6", CarrierEnum::L6});
  bimap.insert({"E1", CarrierEnum::E1});
  bimap.insert({"E5", CarrierEnum::E5});
  bimap.insert({"E6", CarrierEnum::E6});
  bimap.insert({"G1", CarrierEnum::G1});
  bimap.insert({"G1A", CarrierEnum::G1a});
  bimap.insert({"G2", CarrierEnum::G2});
  bimap.insert({"G2A", CarrierEnum::G2a});
  bimap.insert({"B1I", CarrierEnum::B1I});
  bimap.insert({"B1A", CarrierEnum::B1A});
  bimap.insert({"B1C", CarrierEnum::B1C});
  bimap.insert({"B2", CarrierEnum::B2});
  bimap.insert({"B2I", CarrierEnum::B2I});
  bimap.insert({"B2B", CarrierEnum::B2B});
  bimap.insert({"B3", CarrierEnum::B3});
  bimap.insert({"B3A", CarrierEnum::B3A});
  bimap.insert({"S1", CarrierEnum::S1});
  bimap.insert({"U2", CarrierEnum::U2});
  return bimap;
}();

}  // namespace nav::constants