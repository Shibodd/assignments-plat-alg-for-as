#ifndef ARGS_HPP
#define ARGS_HPP

#include "logging.hpp"
namespace cli {

struct args_ty {
  std::string point_cloud_frames_directory = "";
  std::string config_file_path = "";
  logging::LogLevel logLevel = logging::LogLevel::Info;
};

args_ty parse_args(int argc, char* argv[]);

} // namespace cli

#endif // !ARGS_HPP