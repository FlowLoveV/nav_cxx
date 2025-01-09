#pragma once

#include "io/format_options.hpp"
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

  inline void set_format_options(const FormatOptions& options) noexcept { options_ = options; }

  inline const FormatOptions& format_options() const noexcept { return options_; }

  inline FormatOptions& format_options() noexcept { return options_; }

  virtual ~SolutionStream() override = default;

 protected:
  virtual void decode_record(Record& record) override;

  virtual void encode_record(Record& record) override;

  FormatOptions options_{};
};

}  // namespace navp::io::custom
