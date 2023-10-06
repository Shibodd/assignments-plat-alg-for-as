#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace config
{
  struct config_ty
  {
    struct
    {
      float leaf_size_x;
      float leaf_size_y;
      float leaf_size_z;
    } voxel_filtering;
  };

  config_ty parse_config_file(const std::string &path);
} // namespace config

#endif // !CONFIG_HPP