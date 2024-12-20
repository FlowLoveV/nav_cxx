#pragma once

#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <source_location>

#include "utils/macro.hpp"
namespace navp {

using spdlog::level::level_enum;

namespace constants {
inline static std::string LogDirection = "/root/project/nav_cxx/log/nav";
}  // namespace constants

NAVP_EXPORT std::string source_information(std::source_location location = std::source_location::current());

#define nav_log(level, ...)                                                             \
  navp::details::global_formatted_logger->log(level, "{}", navp::source_information()); \
  navp::details::global_pure_logger->log(level, __VA_ARGS__);
#define nav_trace(...) nav_log(spdlog::level::trace, __VA_ARGS__)
#define nav_debug(...) nav_log(spdlog::level::debug, __VA_ARGS__)
#define nav_info(...) nav_log(spdlog::level::info, __VA_ARGS__)
#define nav_warn(...) nav_log(spdlog::level::warn, __VA_ARGS__)
#define nav_error(...) nav_log(spdlog::level::err, __VA_ARGS__)
#define nav_critical(...) nav_log(spdlog::level::critical, __VA_ARGS__)

std::shared_ptr<spdlog::logger> create_common_spdlogger(const std::string& loggerFileName,
                                                        const std::string& loggerName,
                                                        const std::string& direction = constants::LogDirection);

std::shared_ptr<spdlog::logger> create_pure_spdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                      const std::string& direction = constants::LogDirection);

class Logger : public spdlog::logger {
 public:
  using BaseType = spdlog::logger;
  typedef std::shared_ptr<BaseType> LogPtr;

  explicit Logger(std::string name) : spdlog::logger(name) {}

  inline Logger& depoy_sink(spdlog::sink_ptr sink) noexcept {
    this->sinks_.emplace_back(std::move(sink));
    return *this;
  }

  inline Logger& set_level(spdlog::level::level_enum level) noexcept {
    level_ = level;
    return *this;
  }

  inline Logger& flush_on(spdlog::level::level_enum level) noexcept {
    flush_level_ = level;
    return *this;
  }

  inline LogPtr register_self() noexcept {
    if (auto ptr = spdlog::get(name_); ptr) {
      return ptr;
    }
    LogPtr logptr = std::make_shared<BaseType>(std::move(static_cast<BaseType>(*this)));
    spdlog::register_logger(logptr);
    return std::move(logptr);
  }
};

namespace details {

extern NAVP_EXPORT std::shared_ptr<spdlog::logger> global_formatted_logger;

extern NAVP_EXPORT std::shared_ptr<spdlog::logger> global_pure_logger;

}  // namespace details

}  // namespace navp