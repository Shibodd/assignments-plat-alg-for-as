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

  for (size_t i = 0; i < num_particles; ++i) {
    particles.emplace_back(i, 1.0, dist(gen));
  }

  is_initialized = true;
}

/**
 * @brief Initialize the particles using an initial guess of the state.
 * @param state_guess The initial guess
 * @param stddev The 
 * @param num_particles - number of particles
 */
void ParticleFilter::init(Eigen::Vector3d state_guess, Eigen::Vector3d stddev, int num_particles)
{
  particles.clear();
  particles.reserve(num_particles);

  auto dist = make_vector_distribution<std::uniform_real_distribution, double>(state_guess, stddev);

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

  auto dist = make_vector_distribution<std::uniform_real_distribution, double>(Eigen::Vector3d(0,0,0), state_noise);

  for (auto& particle : particles) {
    double new_heading = delta_heading + particle.state.heading();

    Eigen::Vector3d state_delta(
      delta_s * std::cos(new_heading), // x
      delta_s * std::sin(new_heading), // y
      particle.state.heading() + delta_heading // heading
    );

    particle.state.vec() = state_delta + dist(gen);
  }
}

/*
 * TODO
 * This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
 * Input:
 *  mapLandmark   - landmarks of the map
 *  observations  - observations of the car
 * Output:
 *  Associated observations to mapLandmarks (perform the association using the ids)
 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs> &observations)
{
  // TODO
  // TIP: Assign to observations[i].id the id of the landmark with the smallest euclidean distance
}

/*
 * TODO
 * This function updates the weights of each particle
 * Input:
 *  std_landmark   - Sensor noise
 *  observations   - Sensor measurements
 *  map_landmarks  - Map with the landmarks
 * Output:
 *  Updated particle's weight (particles[i].weight *= w)
 */
void ParticleFilter::updateWeights(double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks)
{

  // Creates a vector that stores tha map (this part can be improved)
  std::vector<LandmarkObs> mapLandmark;
  for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
  {
    mapLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f});
  }
  for (int i = 0; i < particles.size(); i++)
  {

    // Before applying the association we have to transform the observations in the global coordinates
    std::vector<LandmarkObs> transformed_observations;
    // TODO: for each observation transform it (transformation function)

    // TODO: perform the data association (associate the landmarks to the observations)

    particles[i].weight = 1.0;
    // Compute the probability
    // The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
    // We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
    for (int k = 0; k < transformed_observations.size(); k++)
    {
      double obs_x, obs_y, l_x, l_y;
      obs_x = transformed_observations[k].x;
      obs_y = transformed_observations[k].y;
      // get the associated landmark
      for (int p = 0; p < mapLandmark.size(); p++)
      {
        if (transformed_observations[k].id == mapLandmark[p].id)
        {
          l_x = mapLandmark[p].x;
          l_y = mapLandmark[p].y;
        }
      }
      // How likely a set of landmarks measurements are, given a prediction state of the car
      double w = exp(-(pow(l_x - obs_x, 2) / (2 * pow(std_landmark[0], 2)) + pow(l_y - obs_y, 2) / (2 * pow(std_landmark[1], 2)))) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      particles[i].weight *= w;
    }
  }
}

/*
 * TODO
 * This function resamples the set of particles by repopulating the particles using the weight as metric
 */
void ParticleFilter::resample()
{

  std::uniform_int_distribution<int> dist_distribution(0, particles.size());
  double beta = 0.0;
  std::vector<double> weights;
  int index = dist_distribution(gen);
  std::vector<Particle> new_particles;

  for (int i = 0; i < particles.size(); i++)
    weights.push_back(particles[i].weight);

  float max_w = *max_element(weights.begin(), weights.end());
  std::uniform_real_distribution<double> uni_dist(0.0, max_w);

  // TODO write here the resampling technique (feel free to use the above variables)
}
