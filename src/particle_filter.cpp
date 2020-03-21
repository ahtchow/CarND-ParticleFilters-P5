#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <map>

#include "helper_functions.h"
#include "particle_filter.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::min_element;
using std::pair;

// Random Generator Engine used for adding noise.
std::default_random_engine gen;

void ParticleFilter::init(
    double x, 
    double y, 
    double theta, 
    double std[]) {
  /**
   * 1. Set the number of particles. Initialize all particles to first position.
   *  (based on estimates of x, y, theta and their uncertainties from GPS).
   * 2. Set all weights to 1.0
   * 3. Add random Gaussian noise to each particle.
   */

  // Create a normal (Gaussian) distribution for x, y and theta.
  normal_distribution<double> gaus_x(x, std[0]);
  normal_distribution<double> gaus_y(y, std[1]);
  normal_distribution<double> gaus_theta(theta, std[2]);
  
  // PARAMETER: Number of Particles.
  num_particles = 20;  

  // Resize weight and particle vector to number of particles.
  weights.resize(num_particles); 
  particles.resize(num_particles);
  
  //For each particle, initialize based on estimates +/= noise.
  //Initialize all weights to 1.0.
  for(int i = 0; i < num_particles; ++i){
    Particle ancestors;
    particles[i].id = i;
    particles[i].x = gaus_x(gen);
    particles[i].y = gaus_y(gen);      
    particles[i].theta = gaus_theta(gen);  
    weights[i]= 1.0;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(
    double delta_t, 
    double std_pos[], 
    double velocity, 
    double yaw_rate){

  // 1. Predict particle positions based on velocity and yaw rate.
	// 2. Add Gaussian noise.

  for(int i = 0; i < num_particles; ++i){

    double coeff = (velocity/yaw_rate);

    // If yaw rate is approx 0.0 update based on kinematics.
    if(fabs(yaw_rate) > 0.001){
      particles[i].x += coeff * ( sin(particles[i].theta + (yaw_rate * delta_t) ) - sin(particles[i].theta) );
      particles[i].y += coeff * ( cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)) );
      particles[i].theta += yaw_rate * delta_t;
    }
    // Else, straight line update.
    else{
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }

    normal_distribution<double> gaus_x(0, std_pos[0]);
    normal_distribution<double> gaus_y(0, std_pos[1]);
    normal_distribution<double> gaus_theta(0, std_pos[2]);

    // Add some gaussian noise to predictions.
    particles[i].x += gaus_x(gen);
    particles[i].y += gaus_y(gen);
    particles[i].theta += gaus_theta(gen);  
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * 
   *   1. Convert observation from VEHICLE'S coordinate system to MAP'S coordinate system. 
   *   You will need to transform between the two systems. (http://planning.cs.uiuc.edu/node99.html)
   * 
   *   2. Update the weights of each particle using a multi-variate Gaussian 
   *   distribution. (https://en.wikipedia.org/wiki/Multivariate_normal_distribution)
   * 
   */

  // Variates used for calculating weight based on multi-variate Gaussian Distribution
  double x_ref, y_ref, mu_x, mu_y, a, b, A, B, gauss_exponent;
  const double sigma_x(std_landmark[0]);
  const double sigma_y(std_landmark[1]);
  const double gauss_coeff(1/(2 * M_PI * sigma_x * sigma_y));

  for(int i = 0; i < particles.size(); ++i){

    double gauss_weight = 1.0;
    
    for(int j= 0; j < observations.size(); ++j){
      
      // VEHICLE'S coordinate system to MAP'S coordinate system. 
      x_ref = (observations[j].x * cos(particles[i].theta)) - (sin(particles[i].theta)*observations[j].y) + particles[i].x;
      y_ref = (observations[j].x * sin(particles[i].theta)) + (cos(particles[i].theta)*observations[j].y) + particles[i].y;
      
      // Find Nearest Neighbours (Landmark)
      vector<pair<Map::single_landmark_s,double>> nearest_neigbour;

      for(int k = 0; k < map_landmarks.landmark_list.size(); ++k){
        
        // Nearest Neighbour based on (x,y) distance.
        double range = dist(x_ref, y_ref, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
        
        // If within sensor range, accept.
        if(range < sensor_range)
          nearest_neigbour.push_back(std::make_pair(map_landmarks.landmark_list[k],range));
      }
      
      if(nearest_neigbour.empty()){
        // If there are no nearest neighbours, push back a large number to reduce weight
        Map::single_landmark_s edge;
        edge.x_f = 100000;
        edge.y_f = 100000;
        edge.id_i = 0;
        nearest_neigbour.push_back(std::make_pair(edge,100000)); 
      }

      int min_dist = 10000000;

      for(int l = 0; l < nearest_neigbour.size(); ++l){
        
        // Find nearest neighbour within min distance in paired list.
        if(nearest_neigbour[l].second < min_dist){
          min_dist = nearest_neigbour[l].second;
          mu_x = nearest_neigbour[l].first.x_f;
          mu_y = nearest_neigbour[l].first.y_f;
        }
      }

      // Calculate Weight: (Multi-Variate Gaussian Distribution)
      a = pow(x_ref - mu_x,2);
      b = pow(y_ref - mu_y,2);;
      A = ( a / (2 * sigma_x * sigma_x));
      B = ( b / (2 * sigma_y * sigma_y));
      gauss_exponent = exp(-1.0*(A+B));

      // For each observation, multiply to the weight.
      gauss_weight *= (gauss_coeff * gauss_exponent); 
    }

    // Set particle weight as the new gaussian weight.
    particles[i].weight = gauss_weight;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * 1. Resample particles with replacement with probability proportional 
   *   to their weight. 
   */

  // Resampled list of particles.
  vector<Particle> new_particles (num_particles);

  // Random Generator
  std::random_device rd;
  std::default_random_engine gen(rd());

  for (int i = 0; i < num_particles; ++i) {

    // Discrete Distribution proportional to weights.
    std::discrete_distribution<int> index(weights.begin(), weights.end());

    // New particles sampled based on probability/importance.
    new_particles[i] = particles[index(gen)];

  }

  // Set new particles.
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // 1. Set associations for x,y                                     
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