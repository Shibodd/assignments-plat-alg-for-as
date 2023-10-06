#ifndef ARGS_HPP
#define ARGS_HPP

#include <string>
#include <iostream>
#include <getopt.h>


#include "logging.hpp"

struct args_ty {
  std::string point_cloud_frames_directory = "";
  logging::LogLevel logLevel = logging::LogLevel::Info;
};


#include "args.hxx"

static args_ty parse_args(int argc, char* argv[]) {
  args::ArgumentParser parser("Assignment 1");
  args::Flag debug(parser, "debug", "Enables debug logging.", { 'd', "debug" });
  args::HelpFlag help(parser, "help", "Display this help message", { 'h', "help" });
  args::Positional<std::string> pcFramesDir(parser, "pointcloud_frames_dir", "The directory which contains the point cloud frames.", args::Options::Required);

  try
  {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help)
  {
    std::cout << parser;
    exit(0);
  }
  catch (args::Error e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    exit(1);
  }

  args_ty ans;
  ans.point_cloud_frames_directory = *pcFramesDir;
  if (debug) {
    ans.logLevel = logging::LogLevel::Debug;
  }

  return ans;
}

#endif // !ARGS_HPP