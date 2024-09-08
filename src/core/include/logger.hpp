#pragma once

#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <source_location>

namespace navp {

using spdlog::level::level_enum;

namespace constants {
constexpr std::string logDirection = "../log/nav/";
constexpr std::string dataDirection = "../log/data/";
}  // namespace constants

std::string sourceInformation(std::source_location location = std::source_location::current());

#define nav_log(level, ...)                                                          \
  navp::details::globalFormattedLogger->log(level, "{}", navp::sourceInformation()); \
  navp::details::globalPureLogger->log(level, __VA_ARGS__);
#define nav_trace(...) nav_log(spdlog::level::trace, __VA_ARGS__)
#define nav_debug(...) nav_log(spdlog::level::debug, __VA_ARGS__)
#define nav_info(...) nav_log(spdlog::level::info, __VA_ARGS__)
#define nav_warn(...) nav_log(spdlog::level::warn, __VA_ARGS__)
#define nav_error(...) nav_log(spdlog::level::err, __VA_ARGS__)
#define nav_critical(...) nav_log(spdlog::level::critical, __VA_ARGS__)

#define nav_data_export(...) navp::details::globalDataLogger->debug(__VA_ARGS__);

// 创建spdlog,这种方法会自动创建一个全局logger,包含两个sink,分别输出到控制台和文件
// 需要注意的是，同名logger注册会获取同一个指针
// level 等级
//    trace
//    debug
//    info
//    warn
//    err
//    critical
// dailysink default level debug (dailysink每日2:00am更新)
// console   default level debug
// logger    default level debug
// logger level 高于其内部sink level时，会覆盖该sink的 level
// ```
// auto logger = spdlog::get(loggerName);
// ```
// logger format
// [2024-04-23 11:58:28.135] [D] [thread 18488] can't find valid ephemeris
std::shared_ptr<spdlog::logger> createCommonSpdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                      const std::string& direction = constants::logDirection);

std::shared_ptr<spdlog::logger> createPureSpdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                    const std::string& direction = constants::logDirection);

// 创建一个数据输出logger，无附带任何格式
std::shared_ptr<spdlog::logger> createDataSpdlogger(const std::string& loggerFileName, const std::string& loggerName,
                                                    const std::string& direction = constants::dataDirection);

namespace details {
// create a global logger, output to file and stdout, with format:
// [2024-04-23 11:58:28.135] [D] [thread 18488] can't find valid ephemeris
// file path -> {workspace}/log/navlogger_$date$.txt
extern std::shared_ptr<spdlog::logger> globalFormattedLogger;

// create a global logger,which output pure message with no more information
extern std::shared_ptr<spdlog::logger> globalPureLogger;

// create a global data logger, output to file, with non-format
// filepath -> {workspace}/data/data_$date$.txt
extern std::shared_ptr<spdlog::logger> globalDataLogger;
}  // namespace details

}  // namespace navp