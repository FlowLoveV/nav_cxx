#include "io/record.hpp"

#include "io/stream.hpp"

namespace navp::io {

Record::~Record() = default;

void Record::put_record(Fstream& stream) const { really_put_record(stream); }

void Record::get_record(Fstream& stream) { really_get_record(stream); }

void Record::really_get_record(Fstream& s) { s.decode_record(*this); }

void Record::really_put_record(Fstream& s) const { s.encode_record(*this); }

}  // namespace navp::io