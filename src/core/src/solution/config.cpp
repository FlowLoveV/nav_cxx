#include "solution/config.hpp"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/gnss.hpp"
#include "solution/config.hpp"

namespace navp::solution {

template <typename T>
using ConfigResult = Result<T, ConfigParseError>;

#define REGISTER_CONFIG_ITEM(name, id) inline constexpr auto name = id;
// project configuration
REGISTER_CONFIG_ITEM(ProjCfg, "meta");
REGISTER_CONFIG_ITEM(TaskNameCfg, "task");        // std::string
REGISTER_CONFIG_ITEM(ProjectNameCfg, "project");  // std::string
REGISTER_CONFIG_ITEM(ExecuteTimeCfg, "time");     // std::string
REGISTER_CONFIG_ITEM(ExecutorCfg, "executor");    // std::string

// solution config
REGISTER_CONFIG_ITEM(SolutionCfg, "solution");
REGISTER_CONFIG_ITEM(SolutionModeCfg, "mode");      // integer
REGISTER_CONFIG_ITEM(SolutionBaseCfg, "base");      // std::string
REGISTER_CONFIG_ITEM(SolutionRoverCfg, "rover");    // std::string
REGISTER_CONFIG_ITEM(SolutionCapacity, "capacity")  // integer
REGISTER_CONFIG_ITEM(SolutionRatio, "ratio")        // float

// output config
REGISTER_CONFIG_ITEM(OutputCfg, "output");
REGISTER_CONFIG_ITEM(OutputDirCfg, "dir");  // std::string

// filter config
REGISTER_CONFIG_ITEM(FilterCfg, "filter")       // std::string
REGISTER_CONFIG_ITEM(FilterItemsCfg, "items");  // table

// station config
REGISTER_CONFIG_ITEM(GlobalStationCfg, "stations");                      // std::string
REGISTER_CONFIG_ITEM(StationObsPathCfg, "observation");                  // std::string
REGISTER_CONFIG_ITEM(StationNavPathCfg, "navigation");                   // std::string
REGISTER_CONFIG_ITEM(StationTypeCfg, "type");                            // std::string
REGISTER_CONFIG_ITEM(StationSourceCfg, "source");                        // std::string
REGISTER_CONFIG_ITEM(StationFixedCfg, "fixed");                          // bool
REGISTER_CONFIG_ITEM(StationRefPosStyleCfg, "reference_position_style")  // std::string
REGISTER_CONFIG_ITEM(StationRefPosCfg, "reference_position");            // std::string
REGISTER_CONFIG_ITEM(StationFrequencyCfg, "frequency");                  // integer
REGISTER_CONFIG_ITEM(StationTropCfg, "trop");                            // integer
REGISTER_CONFIG_ITEM(StationIonoCfg, "iono");                            // integer
REGISTER_CONFIG_ITEM(StationRandomCfg, "random");                        // integer
REGISTER_CONFIG_ITEM(StationCodesCfg, "enabled_codes");                  // table
REGISTER_CONFIG_ITEM(StationLoggerCfg, "logger_name");                   // std::string
REGISTER_CONFIG_ITEM(StationCapacityCfg, "capacity")                     // integer

// logger config
REGISTER_CONFIG_ITEM(GlobalLoggerCfg, "logger");                     // std::string
REGISTER_CONFIG_ITEM(LoggerNameCfg, "name");                         // std::string
REGISTER_CONFIG_ITEM(LoggerConsoleEnableCfg, "enable_console");      // bool
REGISTER_CONFIG_ITEM(LoggerConsoleLevelCfg, "console_level");        // integer
REGISTER_CONFIG_ITEM(LoggerConsolePatternCfg, "console_pattern");    // std::string
REGISTER_CONFIG_ITEM(LoggerFileCfg, "file");                         // table
REGISTER_CONFIG_ITEM(LoggerFileLevelCfg, "level")                    // integer
REGISTER_CONFIG_ITEM(LoggerFileEnableMutCfg, "enable_multithread");  // bool
REGISTER_CONFIG_ITEM(LoggerFilePatternCfg, "pattern");               // std::string
REGISTER_CONFIG_ITEM(LoggerFilePathCfg, "path");                     // std::string
REGISTER_CONFIG_ITEM(LoggerFileTypeCfg, "type");                     // integer

#undef REGISTER_CONFIG_ITEM

using navp::io::rinex::RinexStream;
using CodeMap = navp::sensors::gnss::CodeMap;
using spdlog::level::level_enum;

auto get_node(const NavConfigManger* config, std::string_view top_key,
              std::string_view second_key) noexcept -> ConfigResult<const toml::node*> {
  auto first_node = (*config)[top_key];
  if (!first_node) [[unlikely]] {
    return ConfigParseError(std::format("No top-level key \'{}\' found at {}", top_key, config->path()));
  }
  auto second_node = first_node[second_key];
  if (!second_node) [[unlikely]] {
    return ConfigParseError(
        std::format("No second-level key \'{}\' under \'{}\' found at {}", second_key, top_key, config->path()));
  }
  return second_node.node();
}

auto get_child_node(const toml::node* node, std::string_view key) noexcept -> ConfigResult<const toml::node*> {
  auto child_node = (*node->as_table())[key];
  if (!child_node) [[unlikely]] {
    return ConfigParseError(std::format("No child key \'{}\' under \'{}\'", key, node->source()));
  }
  return child_node.node();
}

template <io::nav_fstream_type StreamType>
auto get_stream(const toml::node* node, std::shared_ptr<spdlog::logger> logger = nullptr) noexcept
    -> ConfigResult<std::unique_ptr<io::Fstream>> {
  if (!node->is_string()) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be a string", node->source()));
  }
  // todo File type detection content should be added here

  auto rnx_stream = std::make_unique<StreamType>(node->as_string()->get(), std::ios::in, logger);
  return std::move(rnx_stream);
}

auto get_nav_record(const toml::node* node, std::shared_ptr<spdlog::logger> logger = nullptr) noexcept
    -> ConfigResult<std::list<GnssNavRecord>> {
  if (!node->is_array()) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be a array", node->source()));
  }
  // todo File type detection content should be added here

  auto ary = node->as_array();
  std::list<GnssNavRecord> result;
  for (auto it = ary->begin(); it != ary->end(); ++it) {
    if (!it->is_string()) [[unlikely]] {
      return ConfigParseError(std::format("Parse error at {}, should be a string", it->source()));
    }
    GnssNavRecord record;
    RinexStream nav_stream(it->as_string()->get(), std::ios::in, logger);
    record.get_record(nav_stream);
    result.emplace_back(std::move(record));
  }
  return result;
}

auto get_coordinate(utils::CoordSystemEnum out_style, const toml::node* node_style,
                    const toml::node* node_coord) noexcept -> ConfigResult<utils::NavVector3f64> {
  if (out_style == utils::CoordSystemEnum::ENU) {
    return ConfigParseError(std::format("The reference position coordinate can't be ENU"));
  }
  if (!node_style->is_integer()) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be a integer", node_style->source()));
  }
  if (!node_coord->is_array()) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be a array", node_coord->source()));
  }
  auto in_style = node_style->as_integer()->get();
  if (in_style < 0 || in_style >= 2) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be 0, 1 or 2", node_style->source()));
  }
  auto coord = node_coord->as_array();
  utils::NavVector3f64 result;
  for (u8 i = 0; i < 3; ++i) {
    if (!coord->get(i)->is_number()) [[unlikely]] {
      return ConfigParseError(std::format("Parse error at {}, should be a number", coord->get(i)->source()));
    }
    result[i] = coord->get(i)->as_floating_point()->get();
  }
  switch (in_style) {
    // xyz
    case 0: {
      if (out_style == utils::CoordSystemEnum::BLH) {
        utils::CoordinateXyz xyz(result);
        result = xyz.to_blh();
      }
      break;
    }
    // blh
    case 1: {
      if (out_style == utils::CoordSystemEnum::XYZ) {
        utils::CoordinateBlh blh(result);
        result = blh.to_xyz();
      }
      break;
    }
    // impossible branch
    default:
      break;
  }
  return std::move(result);
}

template <typename T>
auto get_integer_as(const toml::node* node) noexcept -> ConfigResult<T> {
  if (!node->is_integer()) {
    return ConfigParseError(std::format("Parsing error at {}, should be a integer", node->source()));
  }
  return static_cast<T>(node->as_integer()->get());
}

template <typename T>
auto get_as(const toml::node* node) noexcept -> ConfigResult<T> {
  if (!node->is<T>()) {
    return ConfigParseError(std::format("Parsing error at {}, should be a {}", node->source(), typeid(T).name()));
  }
  return T(*node->as<T>());
}

auto get_enabled_codes(const toml::node* node) noexcept -> ConfigResult<CodeMap> {
  CodeMap code_map;
  for (const auto& [sys, code] : *node->as_table()) {
    ConstellationEnum constellation = sensors::gnss::Constellation::form_str(sys.str().data()).unwrap().id;
    if (auto code_list = code.as_array()) [[likely]] {
      for (auto it = code_list->begin(); it != code_list->end(); ++it) {
        // cheeck if string
        if (!it->is_string()) [[unlikely]] {
          return ConfigParseError(std::format("Parsing error at {}, should be a string", it->source()));
        }
        auto optional_code = magic_enum::enum_cast<ObsCodeEnum>(it->as_string()->get());
        // check if vaild code
        if (!optional_code) [[unlikely]] {
          return ConfigParseError(std::format("Can't parse {} to ObsCodeEnum", it->as_string()->get()));
        }
        code_map[constellation].insert(*optional_code);
      }
    } else {
      return ConfigParseError(std::format("Parsing error at {}, should be a array", code_list->source()));
    }
  }
  return std::move(code_map);
}

template <typename T>
auto get_table_as(const toml::node* node) noexcept -> ConfigResult<std::vector<T>> {
  if (!node->is_array()) {
    return ConfigParseError(std::format("Parsing error at {}, should be a array", node->source()));
  }
  std::vector<T> result;
  for (const auto& item : *node->as_array()) {
    if (auto res = item.as<T>()) {
      result.emplace_back(*res);
    } else {
      return ConfigParseError(std::format("Parsing error at {}, should be a {}", item.source(), typeid(T).name()));
    }
  }
  return std::move(result);
}

template <>
auto get_table_as<std::string_view>(const toml::node* node) noexcept -> ConfigResult<std::vector<std::string_view>> {
  if (!node->is_array()) {
    return ConfigParseError(std::format("Parsing error at {}, should be a array", node->source()));
  }
  std::vector<std::string_view> result;
  for (const auto& item : *node->as_array()) {
    if (item.is_string()) {
      result.emplace_back(item.as_string()->get());
    } else {
      return ConfigParseError(std::format("Parsing error at {}, should be a string", item.source()));
    }
  }
  return std::move(result);
}

/*
 * logger config reading
 */
struct FileLoggerConfig {
  u8 type;
  bool enable_multithread;
  spdlog::level::level_enum level;
  std::string pattern;
  std::string path;

  spdlog::sink_ptr create_file_sink() const noexcept {
    switch (type) {
      case 0:
        return create_basic_file_sink();
      case 1:
        return create_daily_file_sink();
      default:
        return spdlog::sink_ptr();
    }
  }

  spdlog::sink_ptr create_basic_file_sink() const noexcept {
    spdlog::sink_ptr sink;
    if (enable_multithread) {
      sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path);
    } else {
      sink = std::make_shared<spdlog::sinks::basic_file_sink_st>(path);
    }
    sink->set_level(level);
    sink->set_pattern(std::move(pattern));
    return std::move(sink);
  }

  spdlog::sink_ptr create_daily_file_sink() const noexcept {
    spdlog::sink_ptr sink;
    if (enable_multithread) {
      sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(path, 2, 0);
    } else {
      sink = std::make_shared<spdlog::sinks::daily_file_sink_st>(path, 2, 0);
    }
    sink->set_level(level);
    sink->set_pattern(std::move(pattern));
    return std::move(sink);
  }
};

struct LoggerConfig {
  bool enable_console;
  spdlog::level::level_enum console_level, flush_on_level;
  std::string name;
  std::string console_pattern;
  std::vector<FileLoggerConfig> file_loggers;

  std::shared_ptr<spdlog::logger> create_logger() const noexcept {
    Logger logger(name);
    // depoy console
    if (enable_console) {
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(console_level);
      console_sink->set_pattern(std::move(console_pattern));
      logger.depoy_sink(std::move(console_sink));
    }
    // depoy files
    for (auto& file_cfg : file_loggers) {
      logger.depoy_sink(file_cfg.create_file_sink());
    }
    return logger.set_level(spdlog::level::trace).flush_on(flush_on_level).register_self();
  }
};

auto get_logger(const toml::node* node) noexcept -> ConfigResult<std::shared_ptr<spdlog::logger>> {
  LoggerConfig log_cfg;
  if (auto _log = node->as_table()) {
    log_cfg.name = get_as<std::string>(get_child_node(_log, LoggerNameCfg).unwrap_throw()).unwrap_throw();
    log_cfg.enable_console = get_as<bool>(get_child_node(_log, LoggerConsoleEnableCfg).unwrap_throw()).unwrap_throw();
    log_cfg.console_level =
        get_integer_as<level_enum>(get_child_node(_log, LoggerConsoleLevelCfg).unwrap_throw()).unwrap_throw();
    log_cfg.console_pattern =
        get_as<std::string>(get_child_node(_log, LoggerConsolePatternCfg).unwrap_throw()).unwrap_throw();

    if (auto file_log_array = _log->get_as<toml::array>(LoggerFileCfg)) {
      log_cfg.file_loggers.reserve(file_log_array->size());
      for (const auto& file_log : *file_log_array) {
        if (auto _file = file_log.as_table()) {
          FileLoggerConfig file_logger_config;
          auto file_node = std::addressof(file_log);
          file_logger_config.level =
              get_integer_as<level_enum>(get_child_node(file_node, LoggerFileLevelCfg).unwrap_throw()).unwrap_throw();
          file_logger_config.enable_multithread =
              get_as<bool>(get_child_node(file_node, LoggerFileEnableMutCfg).unwrap_throw()).unwrap_throw();
          file_logger_config.pattern =
              get_as<std::string>(get_child_node(file_node, LoggerFilePatternCfg).unwrap_throw()).unwrap_throw();
          file_logger_config.path =
              get_as<std::string>(get_child_node(file_node, LoggerFilePathCfg).unwrap_throw()).unwrap_throw();
          file_logger_config.type =
              get_integer_as<u8>(get_child_node(file_node, LoggerFileTypeCfg).unwrap_throw()).unwrap_throw();
          log_cfg.file_loggers.emplace_back(file_logger_config);
        }
      }
    }
  } else {
    return ConfigParseError(std::format("Parsing error at {}, should be a table", node->source()));
  }
  return log_cfg.create_logger();
}

NavConfigManger::NavConfigManger(std::string_view cfg_path) : toml::parse_result(toml::parse_file(cfg_path)) {}

std::string_view NavConfigManger::path() const noexcept { return *this->source().path; }

auto NavConfigManger::solution_mode() const noexcept -> SolutionModeEnum {
  auto node = get_node(this, SolutionCfg, SolutionModeCfg).unwrap_throw();
  return get_integer_as<SolutionModeEnum>(node).unwrap_throw();
}

auto NavConfigManger::algorithm() const noexcept -> algorithm::AlgorithmEnum {
  auto node = get_node(this, SolutionCfg, SolutionModeCfg).unwrap_throw();
  return get_integer_as<algorithm::AlgorithmEnum>(node).unwrap_throw();
}

auto NavConfigManger::task_name() const noexcept -> std::string {
  auto node = get_node(this, ProjCfg, TaskNameCfg).unwrap_throw();
  return solution::get_as<std::string>(node).unwrap_throw();
}

auto NavConfigManger::proj_name() const noexcept -> std::string {
  auto node = get_node(this, ProjCfg, ProjectNameCfg).unwrap_throw();
  return solution::get_as<std::string>(node).unwrap_throw();
}

auto NavConfigManger::executor_name() const noexcept -> std::string {
  auto node = get_node(this, ProjCfg, ExecutorCfg).unwrap_throw();
  return solution::get_as<std::string>(node).unwrap_throw();
}

auto NavConfigManger::executor_time() const noexcept -> EpochUtc {
  auto node = get_node(this, ProjCfg, ExecuteTimeCfg).unwrap_throw();
  auto str = solution::get_as<std::string>(node).unwrap_throw();
  return EpochUtc::from_str("%Y-%m-%d %H:%M:%S", str.c_str()).unwrap_throw();
}

auto NavConfigManger::base_station(bool enabled_mt) const noexcept -> std::shared_ptr<sensors::gnss::GnssHandler> {
  auto node = get_node(this, SolutionCfg, SolutionBaseCfg).unwrap_throw();
  auto station_name = solution::get_as<std::string>(node).unwrap_throw();
  if (enabled_mt) {
    return navp::GlobalConfig::get_station_mt(station_name);
  } else {
    return navp::GlobalConfig::get_station_st(station_name);
  }
}

auto NavConfigManger::rover_station(bool enabled_mt) const noexcept -> std::shared_ptr<sensors::gnss::GnssHandler> {
  auto node = get_node(this, SolutionCfg, SolutionRoverCfg).unwrap_throw();
  auto station_name = solution::get_as<std::string>(node).unwrap_throw();
  if (enabled_mt) {
    return navp::GlobalConfig::get_station_mt(station_name);
  } else {
    return navp::GlobalConfig::get_station_st(station_name);
  }
}

auto NavConfigManger::logger() const noexcept -> std::shared_ptr<spdlog::logger> {
  auto node = get_node(this, SolutionCfg, GlobalLoggerCfg).unwrap_throw();
  auto logger_name = solution::get_as<std::string>(node).unwrap_throw();
  return GlobalConfig::get_logger(logger_name);
}

auto NavConfigManger::capacity() const noexcept -> i32 {
  auto node = get_node(this, SolutionCfg, SolutionCapacity).unwrap_throw();
  return get_integer_as<i32>(node).unwrap_throw();
}

auto NavConfigManger::filters() const noexcept -> std::vector<std::string_view> {
  auto node = get_node(this, FilterCfg, FilterItemsCfg);
  return get_table_as<std::string_view>(node.unwrap_throw()).unwrap_throw();
}

auto NavConfigManger::ratio() const noexcept -> f32 {
  auto node = get_node(this, SolutionCfg, SolutionRatio).unwrap_throw();
  return static_cast<f32>(solution::get_as<double>(node).unwrap_throw());
}

auto NavConfigManger::output_dir() const noexcept -> std::string {
  auto node = get_node(this, OutputCfg, OutputDirCfg).unwrap_throw();
  return solution::get_as<std::string>(node).unwrap_throw();
}

}  // namespace navp::solution

namespace navp {

using namespace solution;
using namespace sensors::gnss;

// initialize static members
std::string GlobalConfig::config_path_;
std::once_flag GlobalConfig::flag_;
NavConfigManger GlobalConfig::config_;
std::mutex GlobalConfig::mutex_;
std::unordered_map<std::string, std::shared_ptr<sensors::gnss::GnssHandler>> GlobalConfig::st_station_handler_map_;
std::unordered_map<std::string, std::shared_ptr<sensors::gnss::GnssHandler>> GlobalConfig::mt_station_handler_map_;

void GlobalConfig::initialize(std::string config_path) noexcept { config_path_ = config_path; }

solution::NavConfigManger& GlobalConfig::_get_instance() noexcept {
  std::call_once(flag_, [&]() { config_ = NavConfigManger(config_path_); });
  return config_;
}

std::unique_ptr<GnssPayload> GlobalConfig::_get_station_handler(std::string_view station_name) noexcept {
  auto& config = _get_instance();
  auto station_node = get_node(&config_, GlobalStationCfg, station_name).unwrap_throw();
  auto station = std::make_unique<GnssPayload>();

  /*
   * logger
   */
  {
    auto logger_node = get_child_node(station_node, StationLoggerCfg).unwrap_throw();
    station->logger_ = _get_logger(get_as<std::string>(logger_node).unwrap_throw());
  }

  auto& logger = station->logger_;

  /*
   * station settings
   */
  {
    station->settings_ = std::make_unique<GnssSettings>();
    auto& settings = *station->settings_;
    // trop model
    auto trop_node = get_child_node(station_node, StationTropCfg).unwrap_throw();
    settings.trop = get_integer_as<TropModelEnum>(trop_node).unwrap_throw();
    // iono model
    auto iono_node = get_child_node(station_node, StationIonoCfg).unwrap_throw();
    settings.iono = get_integer_as<IonoModelEnum>(iono_node).unwrap_throw();
    // random model
    auto random_node = get_child_node(station_node, StationRandomCfg).unwrap_throw();
    settings.random = get_integer_as<RandomModelEnum>(random_node).unwrap_throw();
    // capacity
    auto capacity_node = get_child_node(station_node, StationCapacityCfg).unwrap_throw();
    settings.capacity = get_integer_as<i32>(capacity_node).unwrap_throw();
    // enabled obs codes
    auto code_node = get_child_node(station_node, StationCodesCfg).unwrap_throw();
    settings.enabled_obs_code = std::make_unique<CodeMap>(get_enabled_codes(code_node).unwrap_throw());
  }

  /*
   * station information
   */
  {
    station->station_info_ = std::make_unique<GnssStationInfo>();
    auto& station_info = *station->station_info_;
    // name
    station_info.name = std::string(station_name);
    // type
    auto type_node = get_child_node(station_node, StationTypeCfg).unwrap_throw();
    station_info.type = get_integer_as<u8>(type_node).unwrap_throw();
    // fixed
    auto fixed_node = get_child_node(station_node, StationFixedCfg).unwrap_throw();
    station_info.fixed = get_as<bool>(fixed_node).unwrap_throw();
    // source
    auto source_node = get_child_node(station_node, StationSourceCfg).unwrap_throw();
    station_info.source = get_integer_as<u8>(source_node).unwrap_throw();
    // healthy
    station_info.healthy = 1;
    // frequency
    auto frequency_node = get_child_node(station_node, StationFrequencyCfg).unwrap_throw();
    station_info.frequency = get_integer_as<u32>(frequency_node).unwrap_throw();
    // ref_pos
    if (station_info.fixed == true) {
      auto ref_style = get_child_node(station_node, StationRefPosStyleCfg).unwrap_throw();
      auto ref_pos = get_child_node(station_node, StationRefPosCfg).unwrap_throw();
      station_info.ref_pos = std::make_unique<utils::CoordinateXyz>(
          get_coordinate(utils::CoordSystemEnum::XYZ, ref_style, ref_pos).unwrap_throw());
    }
  }

  /*
   * station storage
   */
  {
    station->record_ = std::make_unique<GnssRecord>();
    // storage from file source
    auto init_file_source = [&](GnssRecord& storage) {
      // navigation
      auto nav_node = get_child_node(station_node, StationNavPathCfg).unwrap_throw();
      storage.nav = get_nav_record(nav_node, logger).unwrap_throw();
      // observation stream
      auto obs_node = get_child_node(station_node, StationObsPathCfg).unwrap_throw();
      // todo
      // detection observation file type here
      storage.obs_stream = get_stream<RinexStream>(obs_node, logger).unwrap_throw();
      // obs
      storage.obs = std::make_unique<GnssObsRecord>(logger);
      storage.obs->set_storage(station->settings_->capacity);
      if (auto rinex_stream = dynamic_cast<RinexStream*>(storage.obs_stream.get())) {
        rinex_stream->decode_header(*storage.obs);  // read observation header
      }
      // ephemeris solver
      storage.eph_solver = std::make_unique<EphemerisSolver>(logger);
      std::ranges::for_each(storage.nav,
                            [&](const GnssNavRecord& record) { storage.eph_solver->add_ephemeris(record.nav.get()); });
      storage.eph_solver->set_storage(station->settings_->capacity);
    };
    auto& storage = *station->record_;
    switch (station->station_info_->source) {
      // file
      case 0: {
        init_file_source(storage);
        break;
      }
      // network
      case 1: {
        nav_error("not implemented!")
      }
      // serial port
      case 2: {
        nav_error("not implemented!")
      }
    }
  }

  /*
   *station runtime information
   */
  { station->runtime_info_ = std::make_unique<GnssRuntimeInfo>(); }

  logger->info("Station \'{}\' initialized succeed!", station->station_info_->name);
  return station;
}

std::shared_ptr<sensors::gnss::GnssHandler> GlobalConfig::get_station_st(std::string_view station_name) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto it = st_station_handler_map_.find(station_name.data()); it != st_station_handler_map_.end()) {
    return it->second;
  }
  auto station_handler = _get_station_handler(station_name);
  auto gnss_st = std::make_shared<GnssSt>(std::move(*station_handler));
  st_station_handler_map_[station_name.data()] = gnss_st;
  return gnss_st;
}

std::shared_ptr<sensors::gnss::GnssHandler> GlobalConfig::get_station_mt(std::string_view station_name) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto it = mt_station_handler_map_.find(station_name.data()); it != mt_station_handler_map_.end()) {
    return it->second;
  }
  auto station_handler = _get_station_handler(station_name);
  auto gnss_mt = std::make_shared<GnssMt>(std::move(*station_handler));
  mt_station_handler_map_[station_name.data()] = gnss_mt;
  return gnss_mt;
}

std::shared_ptr<spdlog::logger> GlobalConfig::get_logger(std::string_view logger_name) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto logger = spdlog::get(logger_name.data())) {
    return logger;
  }
  auto logger_node = get_node(&_get_instance(), GlobalLoggerCfg, logger_name).unwrap_throw();
  auto logger = solution::get_logger(logger_node).unwrap_throw();
  return logger;
}

std::shared_ptr<spdlog::logger> GlobalConfig::_get_logger(std::string_view logger_name) noexcept {
  if (auto logger = spdlog::get(logger_name.data())) {
    return logger;
  }
  auto logger_node = get_node(&_get_instance(), GlobalLoggerCfg, logger_name).unwrap_throw();
  auto logger = solution::get_logger(logger_node).unwrap_throw();
  return logger;
}

}  // namespace navp