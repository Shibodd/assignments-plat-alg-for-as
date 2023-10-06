#include <iostream>
#include <functional>

#include "../include/config.hpp"
#include "../include/logging.hpp"
#include "../include/ini.hpp"

template <typename T>
static void set_or_leave_default(const mINI::INIMap<std::string> &ini, const std::string &key, T &val)
{
  static auto logger = logging::Logger("config::set_or_leave_default");

  if (ini.has(key))
  {
    const std::string &str_val = ini.get(key);
    logger.debug("Setting %s to %s.", key.c_str(), str_val.c_str());
    
    std::istringstream ss(str_val);
    ss >> val;
    if (!ss.good())
    {
      logger.error("Parse error!");
      exit(1);
    }
  }
  else
  {
    logger.debug("Key %s not found.", key.c_str());
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
    logger.debug("Section %s not found.", name.c_str());
  }
}

namespace config
{
  config_ty parse_config_file(std::string &path)
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

    parse_section(ini, "voxel_filtering", [&ans](mINI::INIMap<std::string> section) {
      set_or_leave_default(section, "leaf_size_x", ans.voxel_filtering.leaf_size_x);
      set_or_leave_default(section, "leaf_size_y", ans.voxel_filtering.leaf_size_y);
      set_or_leave_default(section, "leaf_size_z", ans.voxel_filtering.leaf_size_z); 
    });

    return ans;
  }

} // namespace config