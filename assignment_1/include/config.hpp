#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace config
{
  struct config_ty
  {
    struct
    {
      float leaf_size_x = 0.1f;
      float leaf_size_y = 0.1f;
      float leaf_size_z = 0.1f;
    } voxel_filtering;
  };

  config_ty parse_config_file(std::string &path);
} // namespace config

#endif // !CONFIG_HPP