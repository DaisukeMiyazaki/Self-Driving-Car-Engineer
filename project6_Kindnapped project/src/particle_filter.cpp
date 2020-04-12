/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  // Set the number of particles
	num_particles = 100;  
	// Based on GPS, get x, y and theta data of the vehicle
	double gps_x = x;
	double gps_y = y;
	double gps_theta = theta;
  
	// Uncertainty of GPS 
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
  
	// Creat distributions of possible states of particles  
	std::normal_distribution<double> dist_x(gps_x,std_x);
	std::normal_distribution<double> dist_y(gps_y, std_y);
	std::normal_distribution<double> dist_theta(gps_theta,std_theta);
  
	// Creat details of one single particle 
	Particle st_particle;
  
	for(unsigned int i= 0; i < num_particles ; i++)
	{
		// Set the state of x, y and theta
		st_particle.x = dist_x(gen);
		st_particle.y = dist_y(gen);
		st_particle.theta = dist_theta(gen);
    
		// Set its number and weight of the particle
		st_particle.id = i;
		st_particle.weight = 1.0;
    
		// Append the details to a vector
		particles.push_back(st_particle);
	}
	is_initialized = true;
	// Created a group of num_particles(100) particles, each of which has its own unique state.
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
	// Get a generator that makes a random integer
	std::default_random_engine gen;
  
	// The uncertainties of the vechile's movement
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];
   
	std::normal_distribution<double> dist_x_noise(0., std_x);
	std::normal_distribution<double> dist_y_noise(0., std_y);
	std::normal_distribution<double> dist_theta_noise(0., std_theta);
  
	// Make particles move with the vehicle
	for(unsigned int i = 0 ; i < num_particles ; i++ )
	{
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;
		double predicted_x;
		double predicted_y;
		double predicted_theta;
		if(fabs(yaw_rate) < 0.0001)
		{
			predicted_x = particle_x +  velocity * delta_t * cos(particle_theta);
			predicted_y = particle_y +  velocity * delta_t * sin(particle_theta);
		}
		else // yaw_rate changes
		{
             // deterministic values    
             predicted_x = particle_x + (velocity/yaw_rate) * (sin(fmod(particle_theta + yaw_rate*delta_t, 2*M_PI)) - sin(particle_theta));
             predicted_y = particle_y + (velocity/yaw_rate) * (-cos(fmod(particle_theta + yaw_rate*delta_t, 2*M_PI)) + cos(particle_theta));
             predicted_theta = particle_theta + yaw_rate * delta_t;
             predicted_theta = fmod(predicted_theta, 2*M_PI);
		}
		// Adding noise( AKA uncertainty) to the end state of each particle
		// Set the state of each particle from the distributions
     	predicted_x += dist_x_noise(gen);
     	predicted_y += dist_y_noise(gen);
     	predicted_theta += dist_theta_noise(gen);
		// Set the info of the particle state
     	particles[i].x = predicted_x;
     	particles[i].y = predicted_y;
     	particles[i].theta = predicted_theta;
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
   
	for(unsigned int i = 0 ; i < observations.size() ; i++)
	{
		LandmarkObs obs = observations[i];
		// Initial distance between a single landmark and each point of observations 
		double dis_min = std::numeric_limits<double>::max();
      
		// Assign an invalid value to prevend appendeding an existing index of any landmark 
		// in case all the calculated distances are invalid below
		 
		int min_id = -1;

		for(unsigned int j = 0 ; j < predicted.size() ; j++)
		{
			LandmarkObs pred = predicted[j]; 
			// Distance between a single landmark and each point of observations
			double distance = dist(obs.x, obs.y,pred.x,pred.y);

			if(distance < dis_min)
			{
				// Pick up the smallest distance  between the landmark and one of the observations
				dis_min = distance;
				// Choose the index 
				min_id = pred.id;
			}
		}
		// Append the id od the corresponding landmark 
		observations[i].id = min_id;
	} 
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
 
  // 
	double std_x= std_landmark[0];
	double std_y= std_landmark[1];
  
 	// Convert the vehicle coordinate system to Map coordinate system ( transformed obs list)
 	// The landmarks within the range from the particle 
  
 	// Prepare vectors that have the same structure of Landmark0bs
	LandmarkObs transformed_obs;
	LandmarkObs landmark_within_range;
	vector<LandmarkObs> transformed_obs_list;
	vector<LandmarkObs> landmark_list_within_range;

	for(unsigned int i = 0; i < particles.size(); i++)
	{
 		for(unsigned int j = 0; j < observations.size(); j++)
		{
			// Converting the vehicle coordinate system to Map coordinate system
			transformed_obs.x = particles[i].x + (cos(particles[i].theta)*observations[j].x)
                            -(sin(particles[i].theta)*observations[j].y);
			transformed_obs.y = particles[i].y + (sin(particles[i].theta)*observations[j].x)
                            +(cos(particles[i].theta)*observations[j].y);
			transformed_obs.id = particles[i].id;

			double obs_distance = dist(particles[i].x, particles[i].y, transformed_obs.x, transformed_obs.y);
			if(obs_distance <= sensor_range)
			{
				// Append each structure to an element of the vector
				transformed_obs_list.push_back(transformed_obs);
			}
		}

		for(unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++)
		{
			// Calculate the distance from a single particle to landmarks 
			double distance = dist(particles[i].x, particles[i].y, 
                               map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
			// If the distance is less than sensor_range, as in within the sensor_range
			// Append the id, x and y of the landmark into landmark_list_within_range
			if(distance <= sensor_range)
			{
				landmark_within_range.x = map_landmarks.landmark_list[k].x_f;
				landmark_within_range.y = map_landmarks.landmark_list[k].y_f;
				landmark_within_range.id = map_landmarks.landmark_list[k].id_i;
				landmark_list_within_range.push_back(landmark_within_range);
			}
		}

		dataAssociation(landmark_list_within_range, transformed_obs_list);
		
		double gau_prob; 
		// Reset particles weight to 1.0
		particles[i].weight = 1.0;

		vector<int> asso;
		vector<double> ss_x;
		vector<double> ss_y;
      
		for(int h = 0; h < transformed_obs_list.size(); h++)
		{
			unsigned int landId = transformed_obs_list[h].id - 1;
			double c1 = std_x * std_x;
			double c2 = std_y * std_y;
			double c3 = std_x * std_y;
			
			double residual_1 = pow((transformed_obs_list[h].x - map_landmarks.landmark_list[landId].x_f),2.) / ( 2. * c1);
			double residual_2 = pow((transformed_obs_list[h].y - map_landmarks.landmark_list[landId].y_f),2.) / ( 2. * c2);

			double normalizer = 2. * M_PI * c3;
          
			gau_prob = 1 / normalizer * exp( - (residual_1 + residual_2)); 
			// When the probability of a particle is out of double range, smallen its weight substantially
			// such that the particle woudln't be selected in the next loop
			if(gau_prob == 0.0)
        	{
				particles[i].weight *= 0.00001;
        	}
    		else
        	{
				particles[i].weight *= gau_prob;
        	}
          
			asso.push_back(map_landmarks.landmark_list[landId].id_i);
			ss_x.push_back(transformed_obs_list[h].x);
			ss_y.push_back(transformed_obs_list[h].y);
		}
		// Make a weights distribution over the particles
		weights.push_back(particles[i].weight);
      
		SetAssociations(particles[i], asso, ss_x, ss_y);

		// Clean up
		transformed_obs_list.clear();
		landmark_list_within_range.clear();
	}
	double sum = accumulate(weights.begin(),weights.end(),0.0);

	// normalization of particles weights 
	for(unsigned int r = 0 ; r < particles.size() ; r++ )
	{
		particles[r].weight /= sum;
    }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	std::discrete_distribution<std::size_t> d(weights.begin(),weights.end());
	vector<Particle> new_particles;
	for(int j = 0; j < num_particles; j++)
	{
		// Renew the particles to the new states
		std::size_t index = d(gen);
    	new_particles.push_back(particles[index]);
    }
	// Reset previous particles states and the weight distibution
    particles.clear();
    weights.clear();
    particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}