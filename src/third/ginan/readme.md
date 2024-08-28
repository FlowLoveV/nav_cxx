## ginan库说明

#### Last Update Time
2024/8/26

#### Envirment
- platform ubuntu24.04
- ginan = v3.3.0
- gcc - 14.0.1
- boost - 1.83.0(static libraries)
- mongo-c-driver - 1.27.5
- mongo-cxx-driver - r3.10.2
#### lib Change
- change libssl1.0-dev to libssl3t64
#### source change
1. ginan/common/mqtt.cpp
```
//   double pos[3];
//   ecef2pos(rRec, pos);
auto pos = ecef2pos(rRec);
```


