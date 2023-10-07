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

    struct
    {
      int sac_iterations;
      float distance_threshold;
    } plane_removal;

    struct
    {
      float min_x; float min_y; float min_z;
      float max_x; float max_y; float max_z;
    } crop_cloud;
  };

  config_ty parse_config_file(const std::string &path);
} // namespace config

#endif // !CONFIG_HPP