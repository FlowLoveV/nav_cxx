#include "io/custom/solution_stream.hpp"

#include "solution/solution.hpp"

namespace navp::io::custom {

void SolutionStream::decode_record(Record& record) { nav_error("not implemented") }

void SolutionStream::encode_record(Record& record) { nav_error("not implemented") }

}  // namespace navp::io::custom