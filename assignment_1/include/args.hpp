#ifndef ARGS_HPP
#define ARGS_HPP

#include <string>
#include <iostream>

struct args_ty {
  std::string point_cloud_frames_directory;
};

args_ty parse_args(int argc, char* argv[]) {
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " point_cloud_frames_directory" << std::endl;
    exit(1);
  }
  return (args_ty) {
    .point_cloud_frames_directory = std::string(argv[1])
  };
}

#endif // !ARGS_HPP