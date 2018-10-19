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
	num_particles = 100;

	// The following create a kernel with given mean and standard dev
	// std[] contains Standard deviations for x, y, and theta as gaussian noise parameter
	particles.clear();
	default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	// note that the vector particles, created in header, contains all particles.
	for (int i = 0; i < num_particles; i++) {
		// std::normal_distribution<double> dist_x(x, std[0]);
		// std::normal_distribution<double> dist_y(y, std[1]);
		// std::normal_distribution<double> dist_theta(theta, std[2]);

		// initiate one particle sampling its position from Gaussian kernel
		Particle p;
		p.id = i+1;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		
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
	
	// define gaussian sensor noise
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
	      p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate*dt) - sin(p.theta));
	      p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate*dt));
	      p.theta += yaw_rate * dt;
	    }

	   //  std::normal_distribution<double> noise_x(0, std_pos[0]);
	  	// std::normal_distribution<double> noise_y(0, std_pos[1]);
	  	// std::normal_distribution<double> noise_theta(0, std_pos[2]);


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

 	for (size_t i = 0; i < observations.size(); i++) {
		// LandmarkObs &obs = observations[i];
		// LandmarkObs &pred = predicted[0];
		// initiate closest landmark id and landmark distance to the first predicted landmark
		int id = predicted[0].id;
		double obsx = observations[i].x;
		double obsy = observations[i].y;
		double mindist = dist(obsx, obsy, predicted[0].x, predicted[0].y);

		std::cout << "landmark i, id =  " << i << ", "<< id << std::endl;
		
		for (size_t j = 1; j < predicted.size(); j++) {

			double tdist = dist(obsx, obsy, predicted[j].x, predicted[j].y);

			// LandmarkObs &pred = predicted[j];
			// double tdist = dist(obs.x, obs.y, pred.x, pred.y);

			if (tdist < mindist) {
		    	mindist = tdist;
		    	id = predicted[j].id;
		    	// obs.id = pred.id;
		    	std::cout << "landmark i, id =  " << i << ", "<< id << std::endl;
		    }
		}

		observations[i].id = id;
	}

	// need to check if reference of reference 
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> &observations, Map &map_landmarks) {
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

	std::cout << "Number of particles:  " << num_particles << std::endl;
	std::cout << "Number of observed landmarks:  " << nobs << std::endl;
	std::cout << "Total number of landmarks:  " << nldm << std::endl;

	double totParticleWeight = .0;	

	vector<LandmarkObs> observed; // observed landmarks in global map coordinates
	vector<LandmarkObs> predicted; // possible landmarks associated 

	// Loop each particle in particles, that have been initialized 
	for (int i=0; i < num_particles; i++) {
		Particle& p = particles[i]; // reference p for convenience

		observed.clear();
		predicted.clear();
		// Convert observed landmark coordinates in relation to the location of each particle into global map coordinates 
		for (int j=0; j < nobs; j++) {
			// observations contains observed landmarks
			LandmarkObs &obs = observations[j];
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

		for (int k=0; k < nldm; k++) {
			Map::single_landmark_s lm = map_landmarks.landmark_list[k];
			
			// // Check distance of the particle to each landmark
			// double idist = dist(p.x, p.y, lm.x_f, lm.y_f);
			// // first  landmark filter based distance and compare to sensor_range that is max distance
			// if (idist <= sensor_range) {
			// 	// add landmark object
			// 	LandmarkObs lm_;
			// 	lm_.id = lm.id_i;
			// 	lm_.x = lm.x_f;
			// 	lm_.y = lm.y_f;
			// 	predicted.push_back(lm_);
   			// }
      		LandmarkObs lm_;
			lm_.id = lm.id_i;
			lm_.x = lm.x_f;
			lm_.y = lm.y_f;
			predicted.push_back(lm_);
		}

		// std::cout << "Possible number of landmarks:  " << predicted.size() << std::endl;
		// data association will below will associate each observed landmark 
		// to the one nearest map landmarks (all in global map coordinates) by changing 
		// the observed.id
		dataAssociation(predicted, observed);

		//######################### Calculate weight of the particle ##############################
		// use multivariate Gaussian probability function
		// the measurement has noise govererned by gaussian distribution
		// Use importance sampling

		// initialize weight of the particle
    	double pweight = 1.0;
    	
    	// // double gnorm = (1.0/(2.0 * M_PI * sig_x * sig_y));
    	// double gexpo;
    	// double prob;

    	// Loop each observed landmark
        for (int j = 0; j < nobs; j++) {
        	int idobs = observed[j].id;   	
        	std::cout << "observed landmark id:  " << idobs << std::endl;
	      	for (size_t k = 0; k < predicted.size(); k++) {
	      		int idpred = predicted[k].id;    
	        	if (idobs == idpred) {

	        		LandmarkObs tobs = observed[j]; // observed landmark in map coord
	      			LandmarkObs lm = predicted[k];  // target landmark in map coord

	      			// compute difference of obsered and target
	      			// double tdist = dist(tobs.x, tobs.y, lm.x, lm.y);

	      			// gaussian kernel probability of the landmark measurement
	      			double gexpo = (pow(tobs.x - lm.x, 2))/(2*pow(sig_x, 2)) + pow(tobs.y - lm.y, 2)/(2*pow(sig_y, 2));
					double prob = exp(-gexpo) / (2 * M_PI * sig_x * sig_y);
	        		// gexpo = pow( (tobs.x - lm.x)/sig_x, 2) + pow((tobs.y - lm.y)/sig_y, 2);
	          // 		prob = gnorm * exp(-0.5*gexpo);

					// std::cout << "distx, sigx :  " << tobs.x - lm.x << ","  << sig_x << std::endl;
					// std::cout << "disty, sigy :  " << tobs.y - lm.y << ","  << sig_y << std::endl;

	          		// avoid multiplying by 0
				    if (prob < 1e-4) {
				    	// std::cout << "Warning: probability landmark measurement = " << prob << std::endl;
				    	prob = 1e-4; 
				    }
		
	          		pweight *= prob;   // joint probability of all observing all landmark correctly

	        	}
	        }
	    }

	    p.weight = pweight;
	    totParticleWeight += pweight;

	    // std::cout << "Particle weight:   "<< p.weight << std::endl;
	    // std::cout << "Total Particle Weight =  " << totParticleWeight << std::endl;
	}

	// Normalize the weights to [0, 1] and summing to 1
  	for (int i = 0; i < num_particles; i++) {
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
    std::uniform_int_distribution<int> index_dist(0, num_particles-1);
    std::uniform_real_distribution<double> beta_dist(0, 2*max_weight);
    std::vector<Particle> resampled_particles;

    int cindex = index_dist(ngen);
    double beta = 0.0;
    
    // Wheel draw
    for (int i =0; i< num_particles; i++)
    {
        beta += beta_dist(ngen);
        while (beta > weights[cindex])
        {
            beta -= weights[cindex];
            cindex = (cindex +1) % num_particles;
        }
        resampled_particles.push_back(particles[cindex]);
    }

    particles = resampled_particles;

    std::cout << "Resampled Particle Max Weight = " << max_weight << std::endl;
    std::cout << "Resampled Particles dimension = " << particles.size() << std::endl;

}

Particle ParticleFilter::SetAssociations(Particle& particle, std::vector<int>& associations, 
                                     std::vector<double>& sense_x, std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
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
