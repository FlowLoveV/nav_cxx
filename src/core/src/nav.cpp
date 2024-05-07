// import module time, only valid when ENMODULES is defined
#ifdef ENMODULES
NAV_IMPORT NAV_MODULE_NAME(core);
// if don't enable modules, include files
#else
#include "nav.hpp"
#endif

namespace nav {

std::shared_ptr<spdlog::logger> createCommonSpdlogger(
    const std::string& loggerFileName, const std::string& loggerName) {
  // 查找loggerName,如果已经存在，则直接返回
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
      logDirection + loggerFileName, 2, 0);
  auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  // dailysink为debug等级
  dailySink->set_level(spdlog::level::debug);
  // 设置console为debug等级
  console->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks{dailySink, console};
  auto navLogger =
      std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  // 注册logger
  spdlog::register_logger(navLogger);
  // 设置格式和flush
  navLogger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [thread %t] %v");
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

std::shared_ptr<spdlog::logger> createDataSpdlogger(
    const std::string& loggerFileName, const std::string& loggerName) {
  // 查找loggerName,如果已经存在，则直接返回
  if (auto ptr = spdlog::get(loggerName); ptr) {
    return ptr;
  }
  auto dailySink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
      dataDirection + loggerFileName, 2, 0);
  // dailysink为debug等级
  dailySink->set_level(spdlog::level::debug);
  // 设置console为debug等级
  std::vector<spdlog::sink_ptr> sinks{dailySink};
  auto navLogger =
      std::make_shared<spdlog::logger>(loggerName, begin(sinks), end(sinks));
  navLogger->set_level(spdlog::level::debug);
  // 注册logger
  spdlog::register_logger(navLogger);
  // 设置格式和flush
  navLogger->set_pattern("%v");
  navLogger->flush_on(spdlog::level::debug);
  return navLogger;
}

}  // namespace nav