#include "solution/config.hpp"

#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "io/rinex/rinex_stream.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/config.hpp"
#include "spdlog/sinks/basic_file_sink.h"

namespace navp::solution {

using navp::io::rinex::RinexStream;

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

auto get_obs_stream(const toml::node* node) noexcept -> ConfigResult<std::unique_ptr<io::Stream>> {
  if (!node->is_string()) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be a string", node->source()));
  }
  // todo File type detection content should be added here

  auto rnx_stream = std::make_unique<RinexStream>(node->as_string()->get());
  return std::move(rnx_stream);
}

auto get_nav_record(const toml::node* node) noexcept -> ConfigResult<std::vector<GnssNavRecord>> {
  if (!node->is_array()) [[unlikely]] {
    return ConfigParseError(std::format("Parse error at {}, should be a array", node->source()));
  }
  // todo File type detection content should be added here

  auto ary = node->as_array();
  std::vector<GnssNavRecord> result(ary->size());
  for (auto it = ary->begin(); it != ary->end(); it++) {
    if (!it->is_string()) [[unlikely]] {
      return ConfigParseError(std::format("Parse error at {}, should be a string", it->source()));
    }
    GnssNavRecord record;
    RinexStream nav_stream(it->as_string()->get(), std::ios::in);
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
  for (u8 i = 0; i < 3; i++) {
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
auto get_u8_enum_class(const toml::node* node) noexcept -> ConfigResult<T> {
  if (!node->is_integer()) {
    return ConfigParseError(std::format("Parsing error at {}, should be a integer", node->source()));
  }
  return static_cast<T>(node->as_integer()->get());
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
    log_cfg.name = _log->get_as<std::string>("name")->value_or("empty");
    log_cfg.enable_console = _log->get_as<bool>("enable_console")->value_or(false);
    log_cfg.console_level = static_cast<spdlog::level::level_enum>(_log->get_as<int64_t>("console_level")->value_or(0));
    log_cfg.console_pattern = _log->get_as<std::string>("console_pattern")->value_or("");

    if (auto file_log_array = _log->get_as<toml::array>("file")) {
      log_cfg.file_loggers.reserve(file_log_array->size());
      for (const auto& file_log : *file_log_array) {
        if (auto _file = file_log.as_table()) {
          FileLoggerConfig fileLoggerConfig;
          fileLoggerConfig.level = static_cast<spdlog::level::level_enum>(_file->get_as<int64_t>("level")->value_or(0));
          fileLoggerConfig.enable_multithread = _file->get_as<bool>("enable_multithread")->value_or(false);
          fileLoggerConfig.pattern = _file->get_as<std::string>("pattern")->value_or("");
          fileLoggerConfig.path = _file->get_as<std::string>("path")->value_or("");
          fileLoggerConfig.type = static_cast<u8>(_file->get_as<int64_t>("type")->value_or(0));
          log_cfg.file_loggers.emplace_back(fileLoggerConfig);
        }
      }
    }
  } else {
    return ConfigParseError(std::format("Parsing error at {}, should be a table", node->source()));
  }
  return log_cfg.create_logger();
}

NavConfigManger::NavConfigManger(std::string_view cfg_path) : toml::parse_result(toml::parse_file(cfg_path)) {}

NavConfigManger::~NavConfigManger() noexcept = default;

std::string_view NavConfigManger::path() const noexcept { return *this->source().path; }

auto NavConfigManger::rover_obs() const noexcept -> ConfigResult<std::unique_ptr<io::Stream>> {
  auto node = get_node(this, IoCfg, RoverObsPathCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return get_obs_stream(node.unwrap_unchecked());
}

auto NavConfigManger::base_obs() const noexcept -> ConfigResult<std::unique_ptr<io::Stream>> {
  auto node = get_node(this, IoCfg, BaseObsPathCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return get_obs_stream(node.unwrap_unchecked());
}

auto NavConfigManger::rover_nav() const noexcept -> ConfigResult<std::vector<GnssNavRecord>> {
  auto node = get_node(this, IoCfg, RoverNavPathCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return get_nav_record(node.unwrap_unchecked());
}

auto NavConfigManger::base_nav() const noexcept -> ConfigResult<std::vector<GnssNavRecord>> {
  auto node = get_node(this, IoCfg, RoverNavPathCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return get_nav_record(node.unwrap_unchecked());
}

auto NavConfigManger::base_ref_pos(utils::CoordSystemEnum style) const noexcept -> ConfigResult<utils::NavVector3f64> {
  auto node_style = get_node(this, IoCfg, BaseRefPosStyleCfg);
  if (node_style.is_err()) [[unlikely]] {
    return std::move(node_style.unwrap_err_unchecked());
  }
  auto node_coord = get_node(this, IoCfg, BaseRefPosCfg);
  if (node_coord.is_err()) [[unlikely]] {
    return std::move(node_coord.unwrap_err_unchecked());
  }
  return get_coordinate(style, node_style.unwrap_unchecked(), node_coord.unwrap_unchecked());
}

auto NavConfigManger::rover_ref_pos(utils::CoordSystemEnum style) const noexcept -> ConfigResult<utils::NavVector3f64> {
  auto node_style = get_node(this, IoCfg, RoverRefPosStyleCfg);
  if (node_style.is_err()) [[unlikely]] {
    return std::move(node_style.unwrap_err_unchecked());
  }
  auto node_coord = get_node(this, IoCfg, RoverRefPosCfg);
  if (node_coord.is_err()) [[unlikely]] {
    return std::move(node_coord.unwrap_err_unchecked());
  }
  return get_coordinate(style, node_style.unwrap_unchecked(), node_coord.unwrap_unchecked());
}

auto NavConfigManger::enabled_code() const noexcept -> ConfigResult<CodeTypeMap> {
  auto node_code = get_node(this, ModelCfg, EnabledCodeCfg);
  if (node_code.is_err()) [[unlikely]] {
    return std::move(node_code.unwrap_err_unchecked());
  }
  auto code_table = node_code.unwrap_unchecked();
  if (!code_table->is_table()) [[unlikely]] {
    return ConfigParseError(std::format("Parsing error at {}, should be a table", code_table->source()));
  }
  CodeTypeMap code_map;
  for (const auto& [sys, code] : *code_table->as_table()) {
    ConstellationEnum constellation = sensors::gnss::Constellation::form_str(sys.str().data()).unwrap().id;
    if (auto code_list = code.as_array()) [[likely]] {
      for (auto it = code_list->begin(); it != code_list->end(); it++) {
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

auto NavConfigManger::trop_model() const noexcept -> ConfigResult<TropModelEnum> {
  auto node_trop = get_node(this, ModelCfg, TropModelCfg);
  if (node_trop.is_err()) [[unlikely]] {
    return std::move(node_trop.unwrap_err_unchecked());
  }
  return get_u8_enum_class<TropModelEnum>(node_trop.unwrap());
}

auto NavConfigManger::iono_model() const noexcept -> ConfigResult<IonoModelEnum> {
  auto node_trop = get_node(this, ModelCfg, TropModelCfg);
  if (node_trop.is_err()) [[unlikely]] {
    return std::move(node_trop.unwrap_err_unchecked());
  }
  return get_u8_enum_class<IonoModelEnum>(node_trop.unwrap());
}

auto NavConfigManger::random_model() const noexcept -> ConfigResult<RandomModelEnum> {
  auto node_trop = get_node(this, ModelCfg, TropModelCfg);
  if (node_trop.is_err()) [[unlikely]] {
    return std::move(node_trop.unwrap_err_unchecked());
  }
  return get_u8_enum_class<RandomModelEnum>(node_trop.unwrap());
}

auto NavConfigManger::solution_mode() const noexcept -> ConfigResult<SolutionModeEnum> {
  auto node = get_node(this, ModelCfg, SolutionModeCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return get_u8_enum_class<SolutionModeEnum>(node.unwrap_unchecked());
}

auto NavConfigManger::main_logger() const noexcept -> ConfigResult<std::shared_ptr<spdlog::logger>> {
  auto logger_name = task_name();
  if (logger_name.is_err()) {
    return std::move(logger_name.unwrap_err_unchecked());
  }
  auto node = get_node(this, LoggerCfg, logger_name.unwrap_unchecked().c_str());
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return get_logger(node.unwrap_unchecked());
}

auto NavConfigManger::task_name() const noexcept -> ConfigResult<std::string> {
  auto node = get_node(this, ProjCfg, TaskNameCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return std::string(node.unwrap_unchecked()->as_string()->get());
}

auto NavConfigManger::proj_name() const noexcept -> ConfigResult<std::string> {
  auto node = get_node(this, ProjCfg, ProjectNameCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return std::string(node.unwrap_unchecked()->as_string()->get());
}

auto NavConfigManger::executor_name() const noexcept -> ConfigResult<std::string> {
  auto node = get_node(this, ProjCfg, ExecutorCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  return std::string(node.unwrap_unchecked()->as_string()->get());
}

auto NavConfigManger::executor_time() const noexcept -> ConfigResult<EpochUtc> {
  auto node = get_node(this, ProjCfg, ExecuteTimeCfg);
  if (node.is_err()) [[unlikely]] {
    return std::move(node.unwrap_err_unchecked());
  }
  auto& str = node.unwrap_unchecked()->as_string()->get();
  auto utc = EpochUtc::from_str("%Y-%m-%d %H:%M:%S", str.c_str());
  if (utc.is_err()) {
    return ConfigParseError(utc.unwrap_err_unchecked().what());
  }
  return utc.unwrap_unchecked();
}

}  // namespace navp::solution
