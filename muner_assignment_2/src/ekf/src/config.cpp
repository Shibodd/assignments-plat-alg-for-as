#include <functional>

#include "../include/config.hpp"
#include "../include/logging.hpp"
#include "../include/ini.hpp"
#include <boost/lexical_cast.hpp>

template <typename T>
static void parse_kvp(const mINI::INIMap<std::string> &ini, const std::string &key, T &val)
{
  static auto logger = logging::Logger("config::set_or_leave_default");

  if (ini.has(key))
  {
    const std::string &str_val = ini.get(key);
    logger.debug("%s = %s.", key.c_str(), str_val.c_str());

    try {
      val = boost::lexical_cast<T>(str_val);
    } catch (boost::bad_lexical_cast& e) {
      logger.error("Parse error for key %s: %s.", key, e.what());
      exit(1);
    }
  }
  else
  {
    logger.error("Missing key %s.", key.c_str());
    exit(1);
  }
}

static void parse_section(const mINI::INIStructure &ini, const std::string &name, std::function<void(mINI::INIMap<std::string>)> parser)
{
  static auto logger = logging::Logger("config::parse_section");

  if (ini.has(name))
  {
    logger.debug("Parsing section %s", name.c_str());
    parser(ini.get(name));
  }
  else
  {
    logger.error("Missing section %s.", name.c_str());
    exit(1);
  }
}

namespace config
{
  config_ty parse_config_file(const std::string &path)
  {
    static auto logger = logging::Logger("config::parse_config_file");
    if (path.empty()) {
      logger.info("No configuration file passed. Using default values.");
      return config_ty();
    } else {
      logger.info("Using configuration file %s.", path.c_str());
    }

    mINI::INIFile file(path);
    mINI::INIStructure ini;

    if (!file.read(ini))
    {
      logger.error("INI parsing has failed.");
      exit(1);
    }

    config_ty ans;

    // I'm sorry
    #define PARSE_FIELD(name) parse_kvp(section, #name, section_data.name)
    #define PARSE_SECTION(name, fields) parse_section(ini, #name, [&ans](mINI::INIMap<std::string> section) { \
      auto& section_data = ans.name; \
      fields \
    })

    PARSE_SECTION(laser_variance,
      PARSE_FIELD(x);
      PARSE_FIELD(y);
    );
    PARSE_SECTION(radar_variance,
      PARSE_FIELD(range);
      PARSE_FIELD(heading);
      PARSE_FIELD(radial_velocity);
    );
    PARSE_SECTION(initial_variance,
      PARSE_FIELD(variance);
    );
    PARSE_SECTION(stochastic_noise,
      PARSE_FIELD(x);
      PARSE_FIELD(y);
    );

    return ans;
  }

} // namespace config