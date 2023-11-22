#ifndef CONFIG_PARSE_HPP
#define CONFIG_PARSE_HPP

#include "config.hpp"
#include "logging.hpp"
#include <ros/ros.h>

config_ty cfg;

namespace config_parse {
  template <typename T>
  static void param(const ros::NodeHandle& node, const std::string& name, T& dst, T default_val) {
    static logging::Logger logger("config");
    const char* s = node.param(name, dst, default_val)? "%s: %s" : "%s: %s (default)";
    logger.info(s, name.c_str(), std::to_string(dst).c_str());
  }
  template <typename T>
  static void param_scalar(const ros::NodeHandle& node, const std::string& name, T& dst, T default_val) { param(node, name, dst, default_val); }
  static void param_vector2(const ros::NodeHandle& node, const std::string& name, Eigen::Vector2d& dst, Eigen::Vector2d default_val) { param(node, name + "/x", dst.x(), default_val.x()); param(node, name + "/y", dst.y(), default_val.y()); }
  static void param_vector3(const ros::NodeHandle& node, const std::string& name, Eigen::Vector3d& dst, Eigen::Vector3d default_val) { param(node, name + "/x", dst.x(), default_val.x()); param(node, name + "/y", dst.y(), default_val.y()); param(node, name + "/z", dst.z(), default_val.z()); }
}

#define PARAM_SCALAR_(name, default_val) config_parse::param_scalar(node, "/" #name, cfg.name, default_val)
#define PARAM_VECTOR2(name, default_val) config_parse::param_vector2(node, "/" #name, cfg.name, default_val)
#define PARAM_VECTOR3(name, default_val) config_parse::param_vector3(node, "/" #name, cfg.name, default_val)

static void load_config(const ros::NodeHandle& node) {
  PARAM_VECTOR3(sigma_init, Eigen::Vector3d(0, 0, 0));
  PARAM_VECTOR3(initial_state, Eigen::Vector3d(2.37256, 1.70077, -1.68385));
  PARAM_VECTOR2(min_initial_position, Eigen::Vector2d(-10.0, -20.0));
  PARAM_VECTOR2(max_initial_position, Eigen::Vector2d(10.0, 20.0));
  PARAM_SCALAR_(n_particles, 1000);
  PARAM_SCALAR_(random_initialization, false);

  PARAM_VECTOR3(sigma_pos, Eigen::Vector3d(0.01, 0.01, 0.01));
  PARAM_SCALAR_(dynamic_sigma_pos_gain, 50.0);
  PARAM_VECTOR2(sigma_landmark, Eigen::Vector2d(0.4, 0.4));

  PARAM_SCALAR_(association_max_distance_2, 2.0);
  PARAM_SCALAR_(invalid_association_probability, 1e-5);
}
#undef PARAM



#endif