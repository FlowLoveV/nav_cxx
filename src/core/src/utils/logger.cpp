#include "utils/logger.hpp"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog-inl.h>

#include <memory>
#include <source_location>

#define _normal_logger_pattern "[%Y-%m-%d %H:%M:%S.%e] [%l] [thread %t] %v"
#define _data_logger_pattern "%v"

namespace navp {

std::shared_ptr<spdlog::logger> create_common_spdlogger(const std::string& loggerFileName,
                                                        const std::string& loggerName, const std::string& direction) {
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(direction + loggerFileName);
  auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  dailySink->set_level(spdlog::level::debug);
  console->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks{dailySink, console};
  auto navLogger = std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  spdlog::register_logger(navLogger);
  navLogger->set_pattern(_normal_logger_pattern);
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

std::shared_ptr<spdlog::logger> create_pure_spdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                      const std::string& direction) {
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(direction + loggerFileName);
  auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  dailySink->set_level(spdlog::level::debug);
  console->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks{dailySink, console};
  auto navLogger = std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  spdlog::register_logger(navLogger);
  navLogger->set_pattern(_data_logger_pattern);
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

// output source location using c20 source_location
std::string source_information(std::source_location location) {
  return NAV_FORMAT("{}({}:{}) `{}`", location.file_name(), location.line(), location.column(),
                    location.function_name());
}

namespace details {

std::shared_ptr<spdlog::logger> global_formatted_logger = create_common_spdlogger("navlogger.log", "nav_info_logger");

std::shared_ptr<spdlog::logger> global_pure_logger = create_pure_spdlogger("navlogger.log", "nav_msg_logger");

}  // namespace details

}  // namespace navp