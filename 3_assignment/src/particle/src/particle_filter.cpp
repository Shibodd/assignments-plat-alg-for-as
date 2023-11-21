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
#include "rectangular_lsap.hpp"
#include "gauss.hpp"

#include "logging.hpp"

static std::default_random_engine gen;

/**
 * @brief Randomly initialize the particles.
 * @param min_pos The bottom left corner of the world
 * @param max_pos The upper right corner of the world
 * @param num_particles - number of particles
 */
void ParticleFilter::init_random(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos, size_t num_particles)
{
  TRACE_FN_SCOPE;

  particles.clear();
  particles.reserve(num_particles);

  Eigen::Vector3d min_state(min_pos.x(), min_pos.y(), -M_PI);
  Eigen::Vector3d max_state(max_pos.x(), max_pos.y(), M_PI);
  auto dist = make_vector_distribution<std::uniform_real_distribution, double>(min_state, max_state);

  for (size_t i = 0; i < num_particles; ++i)
  {
    auto state = dist(gen);
    particles.emplace_back(i, 1.0, state);
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
  TRACE_FN_SCOPE;

  particles.clear();
  particles.reserve(num_particles);

  auto dist = make_vector_distribution<std::normal_distribution, double>(state_guess, stddev);

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
  TRACE_FN_SCOPE;
  static const logging::Logger logger("prediction");

  auto dist = make_vector_distribution<std::normal_distribution, double>(Eigen::Vector3d(0, 0, 0), state_noise);

  for (auto &particle : particles)
  {
    Eigen::Vector3d state_delta(
      speed * std::cos(particle.state.heading()),
      speed * std::sin(particle.state.heading()),
      yaw_rate
    );

    particle.state.vec() += dt * (state_delta + dist(gen));
  }
}

static void nn_data_association(
    const std::vector<Eigen::Vector2d> &observations,
    const std::vector<Eigen::Vector2d> &maps,
    std::vector<std::tuple<int, int, double>>& associations,
    std::vector<bool>& obs_is_associated
) {
  TRACE_FN_SCOPE;

  constexpr double INVALID_ASS_DIST_THRESHOLD = 3;
  constexpr double THR = INVALID_ASS_DIST_THRESHOLD * INVALID_ASS_DIST_THRESHOLD;

  associations.clear();
  obs_is_associated.clear();
  obs_is_associated.resize(observations.size(), false);

  // Find the nearest neighbouring observation for each map landmark
  for (size_t map_idx = 0; map_idx < maps.size(); ++map_idx) {
    auto& map = maps[map_idx];
    double min_dist_2 = std::numeric_limits<double>().max();
    size_t min_idx = -1;

    for (size_t obs_idx = 0; obs_idx < observations.size(); ++obs_idx) {
      if (obs_is_associated[obs_idx])
        continue;

      double dist2 = (observations[obs_idx] - map).squaredNorm();
      if (dist2 >= THR || dist2 > min_dist_2)
        continue;

      min_dist_2 = dist2;
      min_idx = obs_idx;
    }

    if (min_idx != -1) {
      obs_is_associated[min_idx] = true;
      associations.emplace_back(min_idx, map_idx, min_dist_2);
    }
  }
}

/**
 * @brief Updates the weights for each particle based on the likelihood of the likelihood of the observed measurements.
 * @param landmark_covariance The sensor covariance matrix
 * @param observations Vector of landmark observations
 * @param map Map class containing map landmarks
 */
void ParticleFilter::updateWeights(
    Eigen::Matrix2d landmark_covariance,
    const std::vector<Eigen::Vector2d> &observed_landmarks,
    const std::vector<Eigen::Vector2d> &map_landmarks)
{
  TRACE_FN_SCOPE;
  MAKE_FN_LOGGER;

  size_t n_obs = observed_landmarks.size();
  size_t n_map = map_landmarks.size();
  size_t n_ass_max = std::min(n_obs, n_map);

  std::vector<Eigen::Vector2d> transformed_observations(n_obs);

  std::vector<bool> obs_is_associated;
  obs_is_associated.reserve(observed_landmarks.size());
  best_associations.reserve(observed_landmarks.size());

  std::vector<std::tuple<int, int, double>> associations;
  associations.reserve(n_ass_max);

  double best_particle_weight = -1000;
  size_t best_particle_idx = -1;
  
  for (size_t i = 0; i < particles.size(); ++i)
  {
    auto& particle = particles[i];
    // Transform observations from local space to global space
    auto l2g_transform = particle.local2global<double>();
    for (size_t i = 0; i < n_obs; ++i)
      transformed_observations[i] = l2g_transform * observed_landmarks[i];

    nn_data_association(transformed_observations, map_landmarks, associations, obs_is_associated);

    /* 
      The new weight is the product of each measurement’s probability density
      in its associated map-centered Multivariate-Gaussian.
    */
    double weight = 1.0;
    for (auto ass : associations)
      weight *= gauss::multivariate_gauss_pdf(
        transformed_observations[std::get<0>(ass)],
        map_landmarks[std::get<1>(ass)],
        landmark_covariance
      );

    size_t invalid_ass_count = n_ass_max - associations.size();

    /*
    Also, what happens if we have unassociated map landmarks (the particle doesn't see them)?
    Use an heuristic: assume the probability of not seeing a landmark is constant.
    
    Important: this way we penalize invalid associations!
    */
    constexpr double INVALID_ASSOCIATION_PROBABILITY = 0.1;
    weight *= std::pow(INVALID_ASSOCIATION_PROBABILITY, invalid_ass_count);

    particle.weight = weight;

    // Compute the best particle
    if (weight > best_particle_weight)
    {
      best_particle_weight = weight;
      best_particle_idx = i;

      // Store the associations, this might be the best particle and we want to render them
      best_associations.swap(associations);
    }
  }

  best_particle_idx_ = best_particle_idx;
}




static inline void naive_wheel_resampling(ParticleFilter& pf) {
  double total_weight = 0.0;
  for (const auto& particle : pf.particles)
    total_weight += particle.weight;

  std::vector<Particle> new_particles;
  new_particles.reserve(pf.particles.size());
  std::uniform_real_distribution<double> dist(0.0, total_weight);

  while (new_particles.size() < pf.particles.size()) {
    double w = dist(gen);

    auto pit = pf.particles.begin();
    auto end = pf.particles.end();
    for (; pit != end; ++pit) {
      w -= (*pit).weight;
      if (w <= 0)
        break;
    }

    new_particles.push_back(*pit);
  }

  pf.particles.swap(new_particles);
}

void ParticleFilter::resample()
{
  TRACE_FN_SCOPE;
  naive_wheel_resampling(*this);
}