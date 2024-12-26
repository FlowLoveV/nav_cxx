#include "io/record.hpp"

#include "io/stream.hpp"

namespace navp::io {

Record::~Record() = default;

void Record::put_record(Fstream& stream) { really_put_record(stream); }

void Record::get_record(Fstream& stream) { really_get_record(stream); }

void Record::really_get_record(Fstream& s) { s.decode_record(*this); }

void Record::really_put_record(Fstream& s) { s.encode_record(*this); }

// std::ostream& operator<<(Fstream& o, const Record& f) {
//   f.put_record(o);
//   return o;
// }

// std::istream& operator>>(Fstream& i, Record& f) {
//   f.get_record(i);
//   return i;
// }

}  // namespace navp::io