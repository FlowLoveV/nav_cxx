#pragma once

#include "io/stream.hpp"

namespace navp::solution {
// forward declaration
struct PvtSolutionRecord;

}  // namespace navp::solution

namespace navp::io::custom {

class SolutionStream;

class NAVP_EXPORT SolutionStream : public Fstream {
 public:
  using Fstream::Fstream;
  virtual ~SolutionStream() override = default;

 protected:
  virtual void decode_record(Record& record) override;

  virtual void encode_record(Record& record) override;

  void encode_pvt_solution_record(solution::PvtSolutionRecord& record);
};

}  // namespace navp::io::custom
