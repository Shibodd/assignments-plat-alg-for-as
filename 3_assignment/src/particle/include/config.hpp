#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>

struct config_ty {
  int n_particles;
  Eigen::Vector3d sigma_init;
  Eigen::Vector3d sigma_pos;
  Eigen::Vector2d sigma_landmark;
  bool random_initialization;
  Eigen::Vector3d initial_state;
  Eigen::Vector2d min_initial_position;
  Eigen::Vector2d max_initial_position;
  double dynamic_sigma_pos_gain;
  double association_max_distance_2;
  double invalid_association_probability;
};

extern config_ty cfg;

#endif