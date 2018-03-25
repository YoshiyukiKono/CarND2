/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

using namespace std;

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};

class ParticleFilter {
	
	// Number of particles to draw
	int num_particles; 
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param num_particles Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	//void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);
	void dataAssociation(std::vector<LandmarkObs> predicted, const std::vector<LandmarkObs>& observations);
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations,
		const Map &map_landmarks);
	
	void normarizeWeights();
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle& particle, const std::vector<int>& associations,
		                     const std::vector<double>& sense_x, const std::vector<double>& sense_y);

	
	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);

	/**
	* initialized Returns whether particle filter is initialized yet or not.
	*/
	const bool initialized() const {
		return is_initialized;
	}

private:

	default_random_engine gen;

	const Map FindTargetLandmarks(const Particle& particle, const double sensor_range, const Map &map_landmarks){
		Map target_landmark;
		for (Map::single_landmark_s landmark : map_landmarks.landmark_list) {
			double dist_p_to_l = dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
			if (dist_p_to_l <= sensor_range) {
				target_landmark.landmark_list.push_back(landmark);
			}
		}
		return target_landmark;
	}

	const Map::single_landmark_s FindNearestLandmark(LandmarkObs &observed, const Map &map_landmarks) const {

		double nearest_dist = 0;
		Map::single_landmark_s nearest_landmark;
		for (int i = 0; i < map_landmarks.landmark_list.size(); i++) {
			Map::single_landmark_s landmark = map_landmarks.landmark_list[i];
			double dist_o_to_l = dist(observed.x, observed.y, landmark.x_f, landmark.y_f);
			if ( nearest_dist == 0 || dist_o_to_l <= nearest_dist) {
				nearest_dist = dist_o_to_l;
				nearest_landmark = landmark;
			}
		}
		return nearest_landmark;
	}

	const void assignLandmarkToObservations(const std::vector<LandmarkObs> &observations, 
			std::vector<int> &associations, std::vector<double> &sense_x, std::vector<double> &sense_y,
			const Map &map_landmarks) const {
		associations.clear();
		sense_x.clear();
		sense_y.clear();
		for (LandmarkObs obs : observations) {
			Map::single_landmark_s nearest_landmark = FindNearestLandmark(obs, map_landmarks);
			associations.push_back(nearest_landmark.id_i);
			sense_x.push_back(obs.x);
			sense_y.push_back(obs.y);
		}
	}

	const std::vector<LandmarkObs> TransformToMapCoordinates(const std::vector<LandmarkObs> &observations, Particle& particle) const {

		double x_part = particle.x;
		double y_part = particle.y;
		double theta = particle.theta;
		
		std::vector<LandmarkObs> map_observations(observations.size());
		for (int i = 0; i < observations.size(); i++) {
			double x_obs = observations[i].x;
			double y_obs = observations[i].y;
			map_observations[i].x = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
			map_observations[i].y = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
		}
		return map_observations;
	}

	const double GauseNorm(double sig_x, double sig_y) const {
		return 1/(2*M_PI * sig_x * sig_y);
	}
	
	const double GauseExp(double x, double y, double mu_x, double mu_y, double sig_x, double sig_y) const {
		return -1*(std::pow((x - mu_x), 2)/(2*std::pow(sig_x, 2)) + std::pow((y - mu_y), 2)/(2*std::pow(sig_y, 2)));
	}
	
	const double GauseWeight(double gause_norm, double gause_exp) const {
		return gause_norm*exp(gause_exp);
	}
	const double CalcWeight(double x, double y, double mu_x, double mu_y, double sig_x, double sig_y) const {
		return GauseWeight(GauseNorm(sig_x, sig_y), GauseExp(x, y, mu_x, mu_y, sig_x, sig_y));
	}

	const double MeasurementProb(Particle& particle, Map map_landmarks, double std_landmark[]) const {

		double prob = 1.0;
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		for (int i = 0; i < particle.associations.size(); i++) {
			double landmark_id = particle.associations[i];
			Map::single_landmark_s landmark = map_landmarks.landmark_list[landmark_id - 1];
			double weight = CalcWeight(particle.sense_x[i], particle.sense_y[i], landmark.x_f, landmark.y_f, sig_x, sig_y);
			prob *= weight;
		}
		return prob;
	}
};

#endif /* PARTICLE_FILTER_H_ */
