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

default_random_engine igen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	num_particles = 400;

	// The following create a kernel with given mean and standard dev
	// std[] contains Standard deviations for x, y, and theta as gaussian noise parameter
	particles.clear();
	weights.clear();
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	// note that the vector particles, created in header, contains all particles.
	for (int i = 0; i < num_particles; i++) {

		// initiate one particle sampling its position from Gaussian kernel
		Particle p;
		p.id = i+1;
		p.x = dist_x(igen);
		p.y = dist_y(igen);
		p.theta = dist_theta(igen);
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
	

  	// Predict position of each particle based on polar to cartesian coordinate conversion formula 
  	for (int i = 0; i < num_particles; i++) {
  		//Particle &p = particles[i];  
  		// p is a reference to particle[i], introduced for name convenience 
	    
	    // calculate new state
	    double theta_prime = (yaw_rate * dt);
		double tx = particles[i].x;
      	double ty = particles[i].y;
      	double ttheta = particles[i].theta;
	    if (fabs(yaw_rate) < 1e-10) { // yaw_rate == 0  
	      	tx += (velocity * dt * cos(ttheta));
	      	ty += (velocity * dt * sin(ttheta));
	    } 
	    else { // yaw_rate!=0
	    	tx += ( (velocity / yaw_rate) * (sin(ttheta + theta_prime) - sin(ttheta)) );
	      	ty += ( (velocity / yaw_rate) * (cos(ttheta) - cos(ttheta + theta_prime)) );
	      	ttheta += theta_prime;
	    }
        std::normal_distribution<double> noise_x(tx, std_pos[0]);
  		std::normal_distribution<double> noise_y(ty, std_pos[1]);
  		std::normal_distribution<double> noise_theta(ttheta, std_pos[2]);
	    
	    // noisy estimates
	    particles[i].x = noise_x(igen);
	    particles[i].y = noise_y(igen);
	    particles[i].theta = noise_theta(igen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	//helper_functions.h defines struct LandmarkObs and method dist 
	// Likely, landmark observation comes from Lidar

 // 	for (size_t i = 0; i < observations.size(); i++) {
	// 	// LandmarkObs &obs = observations[i];
	// 	// LandmarkObs &pred = predicted[0];
	// 	// initiate closest landmark id and landmark distance to the first predicted landmark
	// 	int id = predicted[0].id;
	// 	double obsx = observations[i].x;
	// 	double obsy = observations[i].y;
	// 	double mindist = dist(obsx, obsy, predicted[0].x, predicted[0].y);

	// 	// std::cout << "landmark i, id =  " << i << ", "<< id << std::endl;
		
	// 	for (size_t j = 0; j < predicted.size(); j++) {

	// 		double tdist = dist(obsx, obsy, predicted[j].x, predicted[j].y);

	// 		// LandmarkObs &pred = predicted[j];
	// 		// double tdist = dist(obs.x, obs.y, pred.x, pred.y);

	// 		if (tdist < mindist) {
	// 	    	mindist = tdist;
	// 	    	id = predicted[j].id;
	// 	    	// obs.id = pred.id;
	// 	    	// std::cout << "landmark i, id =  " << i << ", "<< id << std::endl;
	// 	    }
	// 	}

	// 	observations[i].id = id;
	// }

	// Above code has been nested in updateWeights
}

double ParticleFilter::normpdf(double x, double mu, double std) {
	double expo = exp(-0.5*((x-mu)/std)*((x-mu)/std));
	double pdf = (1.0/sqrt(2*M_PI)/std)*expo;
    return pdf;
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

	int nobs = observations.size(); 
	int nldm = map_landmarks.landmark_list.size();

	// std::cout << "Number of particles:  " << num_particles << std::endl;
	// std::cout << "Number of observed landmarks:  " << nobs << std::endl;
	// std::cout << "Total number of landmarks:  " << nldm << std::endl;

	double totParticleWeight = .0;	

	vector<LandmarkObs> observed; // observed landmarks in global map coordinates
	vector<LandmarkObs> possibleObserved; // observed landmarks in global map coordinates
	vector<LandmarkObs> possibeLands; // possible landmarks associated 

	// Loop each particle in particles, that have been initialized 
	for (int i=0; i < num_particles; i++) {
		//Particle& p = particles[i]; // reference p for convenience

		observed.clear();
		possibeLands.clear();
		possibleObserved.clear();
		// Convert observed landmark coordinates in relation to the location of each particle into global map coordinates 
		for (int j=0; j < nobs; j++) {
			// observations contains observed landmarks
			LandmarkObs obs = observations[j];
			double ttheta = particles[i].theta;
			LandmarkObs tobs;
			tobs.id =  obs.id;
			tobs.x =particles[i].x + cos(ttheta)*obs.x - sin(ttheta)*obs.y;
			tobs.y =particles[i].y + sin(ttheta)*obs.x + cos(ttheta)*obs.y;
			observed.push_back(tobs);

		}

    	//########################## Filter landmarks with the nearest neighbor ##################
    	// note that map_landmarks is an instance of class Map (defined in map.h)
    	// with vector element landmark_list and each landmark_list is a struct with element id_i, x_f, y_f (global coordinates)
    	// it contains all the landmarks

		for (int k=0; k < nldm; k++) {
			Map::single_landmark_s lm = map_landmarks.landmark_list[k];
			
			// Check distance of the particle to each landmark
			double idist = dist(particles[i].x, particles[i].y, lm.x_f, lm.y_f);
			
			// Only landmarks within sensor range from particle (vehicle) are considered, allow 1.5*sensor range because of possible measurement error
			if (idist <= 1.5*sensor_range) {
				// add landmark object
				LandmarkObs lm_;
				lm_.id = lm.id_i;
				lm_.x = lm.x_f;
				lm_.y = lm.y_f;
				possibeLands.push_back(lm_);
   			}
		}

		int np = possibeLands.size();
		if (np == 0) cout << "Warning: No landmark possible for a particle i = " << i << endl; // this particle is not possible
		// if (np > 0) dataAssociation(predicted, observed);

		for (int k=0; k < nobs; k++) {
			LandmarkObs oblm = observed[k];
			// Check distance of the particle to each observed landmark
			double idist = dist(particles[i].x, particles[i].y, oblm.x, oblm.y);
			
			// Only landmarks within sensor range from particle (vehicle) are considered, allow 1.5*sensor range because of possible measurement error
			if (idist <= 1.2*sensor_range) {
				// add landmark object
				LandmarkObs oblm_ = oblm;
				possibleObserved.push_back(oblm_);
   			}
   			else {
   				cout << "Warning: Observed landmark coordinates not possible check landmark ID = " << oblm.id << endl;
   			}
		}
		int npo = possibleObserved.size();
		


		// ########################### Associate observed Landmarks with possible predicted and weight particle 
		double minDist = 1.5*sensor_range; 
        double minDist_x = sensor_range;
        double minDist_y = sensor_range;

        double pweight = 1.0;
        for (int j=0; j < npo; j++) {
            for (int k=0; k < np; k++) {
                // Calculate Euclidean distance 
                double tdist = dist( possibleObserved[j].x, possibleObserved[j].y, possibeLands[k].x, possibeLands[k].y);
                if ( (k==0) || (tdist < minDist) ) { // nearest neighbor
                    minDist = tdist;
                    minDist_x = fabs(possibeLands[k].x - possibleObserved[j].x);
                    minDist_y = fabs(possibeLands[k].y - possibleObserved[j].y);
                }
            }
            // Joint likelihood
            pweight *=  normpdf(minDist_x, 0.0, std_landmark[0]) * normpdf(minDist_y, 0.0, std_landmark[1]);
        } 

	    totParticleWeight += pweight;
	    particles[i].weight = pweight;

	    // std::cout << "Particle weight:   "<< p.weight << std::endl;
	    // std::cout << "Total Particle Weight =  " << totParticleWeight << std::endl;
	}

	if (fabs(totParticleWeight < 1e-4)) cout << "Warning: Total weight close to 0" << endl;

	weights.clear();

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

    
    // // Class instances of uniform int and uniform double for index and 2*max weight
    std::uniform_int_distribution<int> index_dist(0, num_particles);
    std::uniform_real_distribution<double> beta_dist(0, 2*max_weight);
    std::vector<Particle> resampled_particles;

    int cindex = index_dist(igen);
    double beta = 0.0;
    
    // Wheel draw
    for (int i=0; i< num_particles; i++)
    {
        beta += beta_dist(igen);
        while (beta > weights[cindex]) {
            beta -= weights[cindex];
            cindex = (cindex +1) % num_particles;
        }
        resampled_particles.push_back(particles[cindex]);
    }

    particles = resampled_particles;

    weights.clear();
    for (int i=0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
	}

    // std::cout << "Resampled Particle Max Weight = " << max_weight << std::endl;
    // std::cout << "Resampled Particles dimension = " << particles.size() << std::endl;

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
