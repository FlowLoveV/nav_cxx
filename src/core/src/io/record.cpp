#include "io/record.hpp"

#include "io/stream.hpp"

namespace navp::io {

Record::~Record() = default;

void Record::put_record(Stream& stream) { really_put_record(stream); }

void Record::get_record(Stream& stream) { really_get_record(stream); }

void Record::really_get_record(Stream& s) { s.decode_record(*this); }

void Record::really_put_record(Stream& s) { s.encode_record(*this); }

// std::ostream& operator<<(Stream& o, const Record& f) {
//   f.put_record(o);
//   return o;
// }

// std::istream& operator>>(Stream& i, Record& f) {
//   f.get_record(i);
//   return i;
// }

}  // namespace navp::io