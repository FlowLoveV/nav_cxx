#include <memory>
#include <string>

#include "macro.hpp"

// export module time, only valid when ENMODULES is defined
#ifdef ENMODULES
NAV_EXPORT NAV_MODULE NAV_MODULE_NAME(core);
NAV_IMPORT STD_MODULE;
// todo non-std headers need to be included
#else
// if don't enable modules, include files
#pragma once
// choose format library
#if FORMATLIB == 0
#include <fmt/format.h>
#elif FORMATLIB == 1
#include <format>
#include <print>
#endif
#include <spdlog/logger.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <string>
#endif

namespace nav {

NAV_EXPORT constexpr std::string logDirection = "../log/";
NAV_EXPORT constexpr std::string dataDirection = "../data/";

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
NAV_EXPORT std::shared_ptr<spdlog::logger> createCommonSpdlogger(
    const std::string& loggerFileName, const std::string& loggerName);

// 创建一个数据输出logger，无附带任何格式
NAV_EXPORT std::shared_ptr<spdlog::logger> createDataSpdlogger(
    const std::string& loggerFileName, const std::string& loggerName);

// create a global logger, output to file and stdout, with format:
// [2024-04-23 11:58:28.135] [D] [thread 18488] can't find valid ephemeris
// file path -> {workspace}/log/navlogger_$date$.txt
NAV_EXPORT auto globalLogger =
    createCommonSpdlogger("navlogger.txt", "navlogger");
// create a global data logger, output to file, with non-format
// filepath -> {workspace}/data/data_$date$.txt
NAV_EXPORT auto globalDataLogger =
    createDataSpdlogger("data.txt", "datalogger");

// get square of T value
template <typename T>
NAV_EXPORT T Square(T t) {
  return t * t;
}

// get cube of T value
template <typename T>
NAV_EXPORT T Cube(T t) {
  return t * t * t;
}

constexpr double ct_sqrt(double x, double current, double prev) {
  return (current == prev) ? current
                           : ct_sqrt(x, 0.5 * (current + x / current), current);
}

}  // namespace nav