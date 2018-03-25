/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 10;//30;
	particles = std::vector<Particle>(num_particles);
	weights = std::vector<double>(num_particles);

	//default_random_engine gen;
	// Standard deviations for x, y, and theta
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std_x);
	
	// TODO: Create normal distributions for y and theta
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	
	for (int i = 0; i < num_particles; ++i) {
		double sample_x, sample_y, sample_theta;
		
		 sample_x = dist_x(gen);
		 sample_y = dist_y(gen);
		 sample_theta = dist_theta(gen);	 
		 
		 particles[i].id = i;
		 particles[i].x = sample_x;
		 particles[i].y = sample_y;
		 particles[i].theta = sample_theta;
		 particles[i].weight = 1;
		 weights[i] = 1;
		 // Print your samples to the terminal.
		 cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	// Standard deviations for x, y, and theta

	const double std_x = std_pos[0];
	const double std_y = std_pos[1];
	const double std_theta = std_pos[2];
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);
	const double yaw_rate_by_delta_t = yaw_rate*delta_t;
	const double velocity_per_yaw_rate = velocity/yaw_rate;
	const double velocity_by_delta_t = velocity * delta_t;

	//default_random_engine gen;
	for (Particle& particle : particles) {

		//predicted state values
		double x_p, y_p;
		double x, y, theta, orientation;
		x = particle.x;
		y = particle.y;
		theta = particle.theta;

		if (fabs(yaw_rate) > 0.001) {
			x_p = x + velocity_per_yaw_rate * (sin(theta + yaw_rate_by_delta_t) - sin(theta));
			y_p = y + velocity_per_yaw_rate * (cos(theta) - cos(theta + yaw_rate_by_delta_t));
			orientation = theta + yaw_rate_by_delta_t;
		} else {
			orientation = theta;
			x_p = x + velocity_by_delta_t * cos(orientation);
			y_p = y + velocity_by_delta_t * sin(orientation);
		}

		particle.x = x_p + dist_x(gen);
		particle.y = y_p + dist_y(gen);
		particle.theta = orientation + dist_theta(gen);	 
	}

}
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, const std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
}
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	weights.clear();
	for (Particle& particle : particles) {

		std::vector<LandmarkObs> map_observations = TransformToMapCoordinates(observations, particle);

		std::vector<int> associations(map_observations.size()); 
		std::vector<double> sense_x(map_observations.size()); 
		std::vector<double> sense_y(map_observations.size());  

		//Map target_landmarks = FindTargetLandmarks(particle, sensor_range, map_landmarks);
		//assignLandmarkToObservations(map_observations, associations, sense_x, sense_y, target_landmarks);
		assignLandmarkToObservations(map_observations, associations, sense_x, sense_y, map_landmarks);

		SetAssociations(particle, associations, sense_x, sense_y);
		double prob = MeasurementProb(particle, map_landmarks, std_landmark);
		particle.weight = prob;
		weights.push_back(prob);
	}

	normarizeWeights();
}

void ParticleFilter::normarizeWeights() {
	weights.clear();
	double weight_sum = 0;
	for (Particle& particle : particles) {
		weight_sum = weight_sum + particle.weight;
	}
	for (Particle& particle : particles) {
		particle.weight = particle.weight/weight_sum;
		weights.push_back(particle.weight);
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::random_device rd;
    std::mt19937 gen(rd());
	//default_random_engine gen;

	std::discrete_distribution<> dst(weights.begin(), weights.end());
	std::vector<Particle> new_particles(num_particles);
	for(int i = 0; i < num_particles; i++) {
		int p_id = dst(gen);
		new_particles[i] = particles[p_id];
	}
	particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();
	
	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}
