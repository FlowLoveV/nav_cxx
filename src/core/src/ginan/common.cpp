#include "ginan/common.hpp"
#include "utils/types.hpp"

using navp::f64;

const f64 ura_eph[] =  ///< URA values (ref [3] 20.3.3.3.1.1)
    {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24, 48, 96, 192, 384, 768, 1536, 3072, 6144};

namespace navp::ginan {

i32 uraToSva(f64 ura) {
  i32 sva = 0;

  if (ura < 0)
    sva = 15;
  else {
    for (sva = 0; sva < 15; sva++)
      if (ura_eph[sva] >= ura) break;
  }

  return sva;
}

i32 sisaToSva(f64 sisa) {
  if (sisa < 0)
    return 255;
  else if (sisa <= 0.49)
    return (i32)((((sisa - 0.0) / 0.01) + 0) + 0.5);
  else if (sisa <= 0.98)
    return (i32)((((sisa - 0.5) / 0.02) + 50) + 0.5);
  else if (sisa <= 1.96)
    return (i32)((((sisa - 1.0) / 0.04) + 75) + 0.5);
  else if (sisa <= 6.00)
    return (i32)((((sisa - 2.0) / 0.16) + 100) + 0.5);
  else
    return 255;
}

}  // namespace navp::ginan