/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016 - Tiffany Huang
	Modified: Oct 2018 - Yujia Hu
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
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 1000;

	// The following create a kernel with given mean and standard dev
	// std[] contains Standard deviations for x, y, and theta as gaussian noise parameter
	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	// note that the vector particles, created in header, contains all particles.
	for (int i = 0; i < num_particles; i++) {
		// initiate one particle sampling its position from Gaussian kernel
		Particle p;
		p.id = i+1;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;

		double sample_x, sample_y, sample_theta;
		
		particles.push_back(p);
		weights.push_back(p.weight);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double dt, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	// define normal distributions for sensor noise
  	
  	std::normal_distribution<double> noise_x(0, std_pos[0]);
  	std::normal_distribution<double> noise_y(0, std_pos[1]);
  	std::normal_distribution<double> noise_theta(0, std_pos[2]);

  	// Random number generator in std::mt19937 std
    default_random_engine ngen;

  	// Predict position of each particle based on polar to cartesian coordinate conversion formula 
  	for (int i = 0; i < num_particles; i++) {
  		Particle &p = particles[i];  
  		// p is a reference to particle[i], introduced for name convenience 
	    
	    // calculate new state
	    if (fabs(yaw_rate) < 1e6) { // yaw_rate == 0  
	      p.x += velocity * dt * cos(p.theta);
	      p.y += velocity * dt * sin(p.theta);
	    } 
	    else { // yaw_rate!=0 
	      p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate*dt) - sin(p.theta));
	      p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate*dt));
	      p.theta += yaw_rate * dt;
	    }

	    // add noise
	    p.x += noise_x(ngen);
	    p.y += noise_y(ngen);
	    p.theta += noise_theta(ngen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	//helper_functions.h defines struct LandmarkObs and method dist 
	// Likely, landmark observation comes from Lidar

	double tdist;
	double mindist;
 	for (int i = 0; i < observations.size(); i++) {
		LandmarkObs &obs = observations[i];
		LandmarkObs &pred = predicted[0];
		// initiate closest landmark id and landmark distance to the first predicted landmark
		obs.id = pred.id;
		mindist = dist(obs.x, obs.y, pred.x, pred.y);
		
		for (int j = 1; j < predicted.size(); j++) {

			LandmarkObs &pred = predicted[j];
			tdist = dist(obs.x, obs.y, pred.x, pred.y);

			if (tdist < mindist) {
		    	mindist = tdist;
		    	obs.id = pred.id;
		    }
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	/*
	Update the weights of each particle using a mult-variate Gaussian distribution. NOTE: 
	1. The observations are given in the VEHICLE'S coordinate system. Your particles are located
		according to the MAP'S coordinate system. You will need to transform between the two systems.
	/	 Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 3.33
	//   http://planning.cs.uiuc.edu/node99.html
	*/

	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];

	int nobs = observations.size(); 
	int nldm = map_landmarks.landmark_list.size();

	double totParticleWeight = .0;	

	// Loop each particle in particles, that have been initialized 
	for (int i=0; i < particles.size(); i++) {
		Particle& p = particles[i]; // reference p for convenience

		// Convert observed landmark coordinates in relation to the location of each particle into global map coordinates 
		vector<LandmarkObs> observed;
		for (int j=0; j < nobs; j++) {
			// observations contains observed landmarks
			const LandmarkObs &obs = observations[j];
			LandmarkObs tobs;
			
			tobs.id =  obs.id;
			tobs.x = p.x + cos(p.theta)*obs.x - sin(p.theta)*obs.y;
			tobs.y = p.y + sin(p.theta)*obs.x + cos(p.theta)*obs.y;

			observed.push_back(tobs);
		}

    	//########################## Filter landmarks with the nearest neighbor ##################
    	// note that map_landmarks is an instance of class Map (defined in map.h)
    	// with vector element landmark_list and each landmark_list is a struct with element id_i, x_f, y_f (global coordinates)
    	// it contains all the landmarks
		vector<LandmarkObs> predicted;
		for (int k=0; k < nldm; k++) {
			const Map::single_landmark_s lm = map_landmarks.landmark_list[k];
			
			// Check distance of the particle to each landmark
			double idist = dist(p.x, p.y, lm.x_f, lm.y_f);
			// double dx = fabs(lm.x_f - p.x);
			// double dy = fabs(lm.y_f - p.y);

			// first  landmark filter based distance and compare to sensor_range that is max distance
			if (idist <= sensor_range) {
				// add landmark
				LandmarkObs lm_;
				lm_.id = lm.id_i;
				lm_.x = lm.x_f;
				lm_.y = lm.y_f;
				predicted.push_back(lm_);
      		}
		}
		// data association will below will associate each observed landmark 
		// to the one nearest map landmarks (all in global map coordinates) by changing 
		// the observed.id
		dataAssociation(predicted, observed);

		//######################### Calculate weight of the particle ##############################
		// use multivariate Gaussian probability function
		// the measurement has noise govererned by gaussian distribution
		// Use importance sampling

		// initialize weight of the particle
    	p.weight = 1.0;
    	
    	double normalize = (1.0/(2.0 * M_PI * sig_x * sig_y));
    	double gexp;
    	double prob;

    	// Loop each observed landmark
        for (int j = 0; j < nobs; j++) {
        	int idobs = observed[j].id;   	
	      	for (int k = 0; k < predicted.size(); k++) {
	      		int idpred = predicted[k].id;    
	        	if (idobs == idpred) {
	        		LandmarkObs tobs = observed[j]; // observed landmark in map coord
	      			LandmarkObs lm = predicted[k];  // target landmark in map coord

	      			// compute difference of obsered and target
	      			// double tdist = dist(tobs.x, tobs.y, lm.x, lm.y);

	      			// gaussian kernel probability of the landmark measurement
	        		gexp = 0.5*( pow((tobs.x - lm.x)/sig_x, 2) + pow((tobs.y - lm.y)/sig_y, 2) );
	          		prob = normalize * exp(-gexp);
	          		p.weight *= prob;   // check algorithm in lecture notes

	          		totParticleWeight += p.weight;
	        	}
	        }
	    }
	}

	// Normalize the weights to [0, 1] and summing to 1
  	for (int i = 0; i < particles.size(); i++) {
  		particles[i].weight /= totParticleWeight;
    	weights[i] = particles[i].weight;
  	}

}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// Use Importance sampling algorithm (resampling wheel method) 
	// * Resampled is basically the Bayesian posterior
	// * The global weights contains already all particle weights normalized

    //Find the max_weight 
    // double max = *max_element(vector.begin(), vector.end()); * because otherwise
    // it will return an iterator
    double max_weight =  *std::max_element(weights.begin(), weights.end());

    // Random number generator in std::mt19937 std
    default_random_engine ngen;
    
    // Class instances of uniform int and uniform double for index and 2*max weight
    std::uniform_int_distribution<int> index_dist(0, num_particles);
    std::uniform_real_distribution<double> beta_dist(0, 2*max_weight);
    std::vector<Particle> resampled_particles;

    int index = index_dist(ngen);
    double beta = 0.0;
    
    // Wheel draw
    for (int i =0; i< num_particles; i++)
    {
        beta += beta_dist(ngen);
        while (beta > weights[index])
        {
            beta -= weights[index];
            index = (index +1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
