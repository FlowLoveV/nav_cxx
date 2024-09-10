#include "utils/logger.hpp"

#include <spdlog/spdlog-inl.h>

#include <memory>
#include <source_location>

#include "utils/macro.hpp"

#define _normal_logger_pattern "[%Y-%m-%d %H:%M:%S.%e] [%l] [thread %t] %v"
#define _data_logger_pattern "%v"

namespace navp {

std::shared_ptr<spdlog::logger> createCommonSpdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                      const std::string& direction) {
  // 查找loggerName,如果已经存在，则直接返回
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(direction + loggerFileName, 2, 0);
  auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  // dailysink为debug等级
  dailySink->set_level(spdlog::level::debug);
  // 设置console为debug等级
  console->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks{dailySink, console};
  auto navLogger = std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  // 注册logger
  spdlog::register_logger(navLogger);
  // 设置格式和flush
  navLogger->set_pattern(_normal_logger_pattern);
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

std::shared_ptr<spdlog::logger> createPureSpdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                    const std::string& direction) {
  // 查找loggerName,如果已经存在，则直接返回
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(direction + loggerFileName, 2, 0);
  auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  // dailysink为debug等级
  dailySink->set_level(spdlog::level::debug);
  // 设置console为debug等级
  console->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks{dailySink, console};
  auto navLogger = std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  // 注册logger
  spdlog::register_logger(navLogger);
  // 设置格式和flush
  navLogger->set_pattern(_data_logger_pattern);
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

std::shared_ptr<spdlog::logger> createDataSpdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                    const std::string& direction) {
  // 查找loggerName,如果已经存在，则直接返回
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(direction + loggerFileName, 2, 0);
  // dailysink为debug等级
  dailySink->set_level(spdlog::level::debug);
  // 设置console为debug等级
  std::vector<spdlog::sink_ptr> sinks{dailySink};
  auto navLogger = std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  // 注册logger
  spdlog::register_logger(navLogger);
  // 设置格式和flush
  navLogger->set_pattern(_data_logger_pattern);
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

// output source location using c20 source_location
std::string sourceInformation(std::source_location location) {
  return NAV_FORMAT("{}({}:{}) `{}`", location.file_name(), location.line(), location.column(),
                    location.function_name());
}

namespace details {
// SPDLOG_LOGGER_CALL(globalLogger, spdlog::level::debug, "this is a debug
// message");
std::shared_ptr<spdlog::logger> globalFormattedLogger = createCommonSpdlogger("navlogger.log", "nav_info_logger");

std::shared_ptr<spdlog::logger> globalPureLogger = createPureSpdlogger("navlogger.log", "nav_msg_logger");

std::shared_ptr<spdlog::logger> globalDataLogger = createDataSpdlogger("data.txt", "datalogger");

}  // namespace details

}  // namespace navp