#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace config
{
  struct config_ty
  {
    struct
    {
      bool enable;
      float leaf_size_x;
      float leaf_size_y;
      float leaf_size_z;
    } voxel_filtering;

    struct
    {
      bool enable;
      int sac_iterations;
      float distance_threshold;
      float ground_offset;
    } ground_removal;

    struct
    {
      bool enable;
      float min_x; float min_y; float min_z;
      float max_x; float max_y; float max_z;
    } crop_cloud;

    struct
    {
      bool enable;
      float min_x; float min_y; float min_z;
      float max_x; float max_y; float max_z;
    } remove_ego_vehicle;

    struct
    {
      float tolerance;
      int min_size;
      int max_size;
    } clustering;

    struct {
      float value1;
      float value2;
    } test;
  };

  config_ty parse_config_file(const std::string &path);
} // namespace config

#endif // !CONFIG_HPP