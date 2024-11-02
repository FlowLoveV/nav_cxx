#pragma once

#include <mutex>
#include <string>
#include "utils/macro.hpp"

namespace navp::solution {

class Task;
class Config;

class NAVP_EXPORT Task {
public:
  Task();
  virtual ~Task() = 0;

  virtual void solve() = 0;
};

class NAVP_EXPORT ConfigTask {
 public:
  ConfigTask();
  virtual ~ConfigTask() = 0;

  ConfigTask(const char* cfg_path);
  ConfigTask(const std::string& cfg_path);

  virtual void solve() = 0;

 protected:
  std::string config_path_;
  Config* config_;
};

class NAVP_EXPORT ConcurrentTask : public ConfigTask {
 public:
  using ConfigTask::ConfigTask;

 protected:
  std::mutex mutex_;
};

}  // namespace navp::solution