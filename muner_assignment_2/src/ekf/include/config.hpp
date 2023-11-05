#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

namespace config
{
  struct config_ty
  {
    struct {
      double variance;
    } initial_variance;

    struct {
      double x;
      double y;
    } laser_variance;

    struct {
      double range;
      double heading;
      double radial_velocity;
    } radar_variance;

    struct {
      double x;
      double y;
    } stochastic_noise;
  };

  config_ty parse_config_file(const std::string &path);
} // namespace config

#endif // !CONFIG_HPP