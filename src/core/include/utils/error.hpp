#pragma once

#include <cstdint>

#include "utils/macro.hpp"

namespace navp::errors {

struct NAVP_EXPORT NavError {
  struct Initalize {
    struct GetArguments {
      enum : uint16_t {
        NotEnoughArguments = 0,
        ErrorArguments,
        UnknownArguments,
      };
    };

    struct Configuration {
      enum : uint16_t {
        NoConfigurationFile = 100,
        ParseConfigurationError,
      };
    };

    struct Io {
      enum : uint16_t {
        ReceiveWrongStream = 300,
      };
      struct Rinex {
        enum : uint16_t {

        };
      };
    };
  };

  struct Utils {
    struct Gnss {
      enum : uint16_t {
        ParseSvStringError = 1000,
        ParseConstellationStringError,
        ParseCarrierStringError,
      };
    };

    struct Time {
      enum : uint16_t {
        ParseEpochError = 1100,
      };
    };

    struct Filter {
      enum : uint16_t {
        ParseOperandError = 1110,
      };
    };
  };

  struct PreProcess {};

  struct Process {
    struct Gnss {
      struct EphemerisSolver {
        enum : uint16_t {
          KeplerIterationOvrflow = 2000,
        };
      };
    };
  };

  struct OutPut {};
};

}  // namespace navp::errors