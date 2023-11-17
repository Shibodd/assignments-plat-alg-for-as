#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle/particle_filter.h"
#include "vector_distribution.hpp"
#include "lsap.hpp"
#include "gauss.hpp"

static std::default_random_engine gen;

/**
 * @brief Randomly initialize the particles.
 * @param min_pos The bottom left corner of the world
 * @param max_pos The upper right corner of the world
 * @param num_particles - number of particles
 */
void ParticleFilter::init_random(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos, size_t num_particles)
{
  particles.clear();
  particles.reserve(num_particles);

  Eigen::Vector3d min_state(min_pos.x(), min_pos.y(), -M_PI);
  Eigen::Vector3d max_state(max_pos.x(), max_pos.y(), M_PI);
  auto dist = make_vector_distribution<std::uniform_real_distribution, double>(min_state, max_state);

  for (size_t i = 0; i < num_particles; ++i)
  {
    particles.emplace_back(i, 1.0, dist(gen));
  }

  is_initialized = true;
}

/**
 * @brief Initialize the particles using an initial guess of the state.
 * @param state_guess The initial guess
 * @param stddev The standard deviation for each coordinate (simplifying assumption, each coordinate is independent => not a multivariate gaussian)
 * @param num_particles - number of particles
 */
void ParticleFilter::init(Eigen::Vector3d state_guess, Eigen::Vector3d stddev, int num_particles)
{
  particles.clear();
  particles.reserve(num_particles);

  auto dist = make_vector_distribution<std::normal_distribution, double>(state_guess, stddev);

  // TODO: 1.0 weight for every particle is probably wrong
  for (int i = 0; i < num_particles; i++)
    particles.emplace_back(i, 1.0, dist(gen));

  is_initialized = true;
}

/**
 * @brief Estimate the particle state after dt seconds.
 * @param dt Time step
 * @param state_noise The noise to add to the new state
 * @param velocity The measured speed
 * @param yaw_rate The measured yaw rate
 */
void ParticleFilter::prediction(double dt, Eigen::Vector3d state_noise, double speed, double yaw_rate)
{
  double delta_s = speed * dt;
  double delta_heading = yaw_rate * dt;

  auto dist = make_vector_distribution<std::normal_distribution, double>(Eigen::Vector3d(0, 0, 0), state_noise);

  for (auto &particle : particles)
  {
    double new_heading = delta_heading + particle.state.heading();

    Eigen::Vector3d state_delta(
        delta_s * std::cos(new_heading),         // x
        delta_s * std::sin(new_heading),         // y
        particle.state.heading() + delta_heading // heading
    );

    particle.state.vec() = state_delta + dist(gen);
  }
}

static Eigen::MatrixXd assignment_cost_matrix(
    const std::vector<Eigen::Vector2d> &observations,
    const std::vector<Eigen::Vector2d> &map,
    const Eigen::Matrix2d& covariance_inverse,
    Eigen::MatrixXd& ans)
{
  int obs_count = observations.size();
  int map_count = map.size();

  // Compute each element of the cost matrix
  for (size_t obs_idx = 0; obs_idx < obs_count; ++obs_idx)
  {
    for (size_t map_idx = 0; map_idx < map_count; ++map_idx)
    {
      double dist = gauss::mahalanobis2(observations[obs_idx], map[map_idx], covariance_inverse);

      // The covariance is invariant across particles => Drop the scaling term from the Multivariate Gaussian PDF
      ans(obs_idx, map_idx) = -std::exp(-dist / 2);
    }
  }

  return ans;
}


/**
 * @brief Updates the weights for each particle based on the likelihood of the likelihood of the observed measurements.
 * @param landmark_covariance The sensor covariance matrix
 * @param observations Vector of landmark observations
 * @param map Map class containing map landmarks
 */
void ParticleFilter::updateWeights(
    Eigen::Matrix2d landmark_covariance_inverse,
    const std::vector<Eigen::Vector2d> &observed_landmarks,
    const std::vector<Eigen::Vector2d> &map_landmarks)
{
  size_t n_obs = observed_landmarks.size();
  std::vector<Eigen::Vector2d> transformed_observations(n_obs);
  Eigen::MatrixXd association_costs(n_obs, n_obs);
  
  for (auto& particle : particles)
  {
    // Transform observations from local space to global space
    auto l2g_transform = particle.local2global<double>();
    for (size_t i = 0; i < n_obs; ++i)
      transformed_observations[i] = l2g_transform * observed_landmarks[i];

    // Compute the cost matrix
    assignment_cost_matrix(transformed_observations, map_landmarks, landmark_covariance_inverse, association_costs);

    // Associate the observations to landmarks
    std::vector<std::pair<int, int>> associations = lsap::solve(association_costs);

    // Square LSAP - there are no unassociated observations
    assert(associations.size() == transformed_observations.size());

    /* 
      The new weight is the product of each measurement’s probability density
      in its associated map-centered Multivariate-Gaussian.
      
      We already computed them in association_costs.
    */
    double weight = 1.0;
    for (auto [obs_idx, map_idx] : associations)
      weight *= association_costs(obs_idx, map_idx); 
    particle.weight = weight;
  }
}


void ParticleFilter::resample()
{
  naive_wheel_resampling(*this);
}

static inline void naive_wheel_resampling(ParticleFilter& pf) {
  double total_weight = 0.0;
  for (const auto& particle : pf.particles)
    total_weight += particle.weight;

  std::vector<Particle> new_particles(pf.particles.size());
  std::uniform_real_distribution<double> dist(0.0, total_weight);

  for (int i = 0; i < new_particles.size(); ++i) {
    double w = dist(gen);

    auto pit = pf.particles.begin();
    while (true) {
      w -= *pit.weight;
      if (w <= 0)
        break;
      ++pit;
    }

    new_particles[i] = *pit;
    w -= *pit.weight;
  }
}
