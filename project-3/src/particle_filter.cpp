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

	num_particles = 30;
	particles = std::vector<Particle>(num_particles);
	weights = std::vector<double>(num_particles);

	default_random_engine gen;
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
	
//cout << "START prediction" << endl;
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	default_random_engine gen;
	for (Particle particle : particles) {
		double sample_x, sample_y, sample_theta;

//cout << "prediction - Particle1" << endl;			
		//predicted state values
    	double x_p, y_p;
    	double x, y, theta, orientation;
    	x = particle.x;
    	y = particle.y;
	    theta = particle.theta;
	    normal_distribution<double> dist_theta(0, std_theta);
		orientation = theta + yaw_rate + dist_theta(gen);
		
//cout << "prediction - Particle2" << endl;
	    x_p = x + velocity * delta_t * cos(orientation);
	    y_p = y + velocity * delta_t * sin(orientation);

		normal_distribution<double> dist_x(x_p, std_x);
		normal_distribution<double> dist_y(y_p, std_y);

//cout << "prediction - Particle3" << endl;
		 sample_x = dist_x(gen);
		 sample_y = dist_y(gen);
		 sample_theta = dist_theta(gen);	 
//cout << "prediction - Particle4" << endl;
		 particle.x = sample_x;
		 particle.y = sample_y;
		 particle.theta = sample_theta;
	}

}
//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, const std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (Particle particle : particles) {
		std::vector<int> associations = std::vector<int>(predicted.size());
		std::vector<double> sense_x = std::vector<double>();
		std::vector<double> sense_y = std::vector<double>();
		SetAssociations(particle, associations, sense_x, sense_y);
	}
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

// NEED TO USE  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]
//double x_sig = std_landmark[0];
//double y_sig = std_landmark[1];
//cout << "START updateWeights" << endl;
	//dataAssociation(predicted, observations);
	weights.clear();
	for (Particle particle : particles) {
		
//cout << "updateWeights - Particle" << endl;	
				
		double x_part = particle.x;
		double y_part = particle.y;
		vector<LandmarkObs> map_observations = TransformToMapCoordinates(observations, x_part, y_part);
		//map_observations = TransformToMapCoordinates(observations, x_part, y_part);
		Map target_landmarks = FindTargetLandmarks(particle, sensor_range, map_landmarks);
		//const double max_dist = CalcMaxDistance(target_landmarks);
		
		std::vector<int> associations(map_observations.size()); 
        std::vector<double> sense_x(map_observations.size()); 
        std::vector<double> sense_y(map_observations.size());  
		
		assignLandmarkToObservations(map_observations, associations, sense_x, sense_y, target_landmarks);//, max_dist);
		//assignLandmarkToObservations(map_observations, target_landmarks, max_dist);
//std::cout << "END assignLandmarkToObservations - associations:" << associations.size() << std::endl;
		particle = SetAssociations(particle, associations, sense_x, sense_y);
//std::cout << "END SetAssociations" << std::endl;
		// TODO: Update the weights of each particle using a mult-variate Gaussian distribution
		//UpdateParticleWeights(particle);
		//  CHECK
		double sensor_noise = 0;
		double prob = MeasurementProb(particle, map_landmarks, sensor_noise, std_landmark);
		particle.weight = prob;
		weights.push_back(prob);
std::cout << "updateWeights - particle.weight:" <<  particle.weight << std::endl;
	}
	// Normarize weight
	//normarizeWeights();
}

void ParticleFilter::normarizeWeights() {
std::cout << "normarizeWeights - particles.size():" <<  particles.size() << std::endl;
	weights.clear();
	for (Particle particle : particles) {
std::cout << "normarizeWeights - weights:" <<  particle.weight << std::endl;
		weights.push_back(particle.weight);
	}
std::cout << "normarizeWeights - weights.size():" <<  weights.size() << std::endl;
	// TODO SOMETHING
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	std::random_device rd;
    std::mt19937 gen(rd());
//std::cout << "resample - weights.begin():" <<  weights[0] << std::endl;
//std::cout << "resample - weights.end():" <<  weights.end() << std::endl;

    std::discrete_distribution<> dst(weights.begin(), weights.end());
    std::vector<Particle> new_particles = std::vector<Particle>(num_particles);
    for(int i = 0; i < num_particles; i++) {
//std::cout << "resample - dst(gen):" <<  dst(gen) << std::endl;
    	Particle particle = particles[dst(gen)];
//std::cout << "resample - particles[dst(gen)].weight:" <<  particle.weight << std::endl;
        new_particles.push_back(particles[dst(gen)]);
    }
///std::cout << "resample - new_particles.size():" <<  new_particles.size() << std::endl;
///std::cout << "resample - new_particles[0].weight:" <<  new_particles[0].weight << std::endl;
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
