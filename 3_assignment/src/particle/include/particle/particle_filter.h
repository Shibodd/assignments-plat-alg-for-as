#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <Eigen/Dense>
#include "particle/state.hpp"

struct Particle {
	state_ty state;

	// Returns the local to global transform for the current particle.
	template <typename T>
	inline Eigen::Transform<T, 2, 1> local2global() const { Eigen::Rotation2D<T>(state.heading()) * Eigen::Translation<T, 2>(state.head(2)); }

	int id;
	double weight; // represents the weight/importance of the particle
	std::vector<int> associations; //stores the associations between measurements and map
	std::vector<double> sense_x;
	std::vector<double> sense_y;

	Particle(int id, double weight, Eigen::Vector3d state) : id(id), state(state), weight(weight) {};
};



class ParticleFilter {
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param M Number of particles
	ParticleFilter() : is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * @brief Initialize the particles using an initial guess of the state.
	 * @param state_guess The initial guess
	 * @param stddev The 
	 * @param num_particles - number of particles
	 */
	void init(Eigen::Vector3d state_guess, Eigen::Vector3d stddev, int num_particles);

	/**
	 * @brief Randomly initialize the particles.
	 * @param min_pos The bottom left corner of the world 
	 * @param max_pos The upper right corner of the world
	 * @param num_particles - number of particles
	 */
	void init_random(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos, size_t num_particles);

	/**
	 * @brief Estimate the particle state after dt seconds.
	 * @param dt Time step
	 * @param state_noise The noise to add to the new state
	 * @param velocity The measured speed
	 * @param yaw_rate The measured yaw rate
	*/
	void prediction(double dt, Eigen::Vector3d state_noise, double speed, double yaw_rate);
	
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
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y);
	
	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);

	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const {
		return is_initialized;
	}
};



#endif /* PARTICLE_FILTER_H_ */
