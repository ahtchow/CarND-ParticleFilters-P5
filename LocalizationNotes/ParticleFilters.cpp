/**
 * @file ParticleFilters.cpp
 * @author Adrian Chow
 * @version 0.1
 * @date 2020-03-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Particle Filters Pseudocode (x(t-1), u, zt)
 *      1. X(t) -> Initialization
 *      2. For m = 1 to M do
 *      3.  Sample x(t) ~p(x(t)|u(t), x(t-1)) -> Prediction
 *      4.  w(t) = p(z(t)|x(t)) -> Update 1
 *      5.  X(t) = X(t) + [ x(t) , w(t) ] -> Update 2
 *      6. end forloop
 *      7. For m = 1 to M do 
 *      8.  draw i with a probability of w(t) -> Resampling 1
 *      9.  add x(t) to X(t) -> Resampling 2
 *     10. end forloop
 *     11. Return X(t)
 * 
 *      Initalization - Estimate our position from GPS input.
 *                      Obviously with some errors.
 * 
 *      Prediction - Add the control input (yaw rate and velocity)
 *                      for all particles.
 * 
 *      Update - Update particle weights using map landmark positions 
 *                      and feature measurements
 * 
 *      Resampling - Draw a new set of particles randomly passed on the 
 *                      weight of existing particles. (Resampling Wheel)
 * 
 *      Return - Return New Particle Set
 * 
 * */


#include <iostream>
#include <random> // Need this for sampling from distributions
#include <cmath>
using namespace std;
using std::normal_distribution;

/**
 * Prints samples of x, y and theta from a normal distribution
 * @param gps_x   GPS provided x position
 * @param gps_y   GPS provided y position
 * @param theta   GPS provided yaw
 */
void Initialization(double gps_x, double gps_y, double theta);
void calculateTransformation();
void calculateParticleWeights();

int main() {
  
  // Set GPS provided state of the car.
  double gps_x = 4983;
  double gps_y = 5029;
  double theta = 1.201;
  
  // Sample from the GPS provided position.
  Initialization(gps_x, gps_y, theta);
  calculateTransformation();
  calculateParticleWeights();
  
  return 0;
}

/**
 * @brief Initialization
 * 
 *  - Decide how many particles?
 *  - Too few will not cover highly likely positions entirely (e.g 3 particles but car takes up 4)
 *  - Too many will slow down filter and prevent localization in real time
 *      
 *  2 Ways to initialize:
 *      1. Sample Uniformely along state-space
 *          - Grid w. 1 particle in each cell
 *          - Not practical, since world to large
 *      2. Using Initial Estimate
 *          - GPS is low accuracy, but can be used to intial.
 *  
 *      We will use method 2.
 */

void Initialization(double gps_x, double gps_y, double theta) {
  std::default_random_engine gen;   // Random engine
  double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta

  //Set standard deviations for x, y, and theta
  std_x = std_y = std_theta = 2.0;


  // These line creates a normal (Gaussian) distribution for x, y and theta.
  normal_distribution<double> dist_x(gps_x, std_x);
  normal_distribution<double> dist_y(gps_y, std_y);
  normal_distribution<double> dist_theta(gps_y, std_y);

  for (int i = 0; i < 3; ++i) {
    double sample_x, sample_y, sample_theta;

    // Sample from these normal distributions like this: 
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);      
    sample_theta = dist_theta(gen);  
     
    // Print your samples estimates to the terminal.
    std::cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " 
              << sample_theta << std::endl;
  }

  return;
}

/**
 * @brief Prediction Step
 *  - For each particle update the particle's location
 *  - using velocity and yaw rate
 *  - w/ Gaussian (Sensor) Noise
 * 
 * Data Association: Nearest Neighbor
 *  - Matching landmark measurements to objects in realworld (maps)
 *  - We can assume two lidar measurements corespond to one object by taking Nearest Neighbour
 *  - Nearest Neighbour: Choose closest lidar measurement to the map landmark measurement
 * What would help nearest neighbor work:
 *  - High signal-to-noise ratio for sensors
 *  - Very accurate motion model
 * Pros of Nearest Neighbour:
 *  - Easy to understand, implement, works well generally
 *  
 * Cons of Nearest Neighbour:
 *  - High Density of Measurements could cause high amount of noise
 *  - Inefficient: O(mxn) -> m - amount of landmarks , n - amount of lidar measurements 
 *  - If vechile position has errors: nearest neighbour will not have intuition to shift
 *  - Radar Measurements: Has accurate bearing measurements, but not in range dimension
 * 
 * 
 **/ 

/**
 * @brief Update Step
 *  - Instead of the feature measurements directly predicting the weight of the car,
 *      the particles will assign weights to each particle.
 *  - Use Gaussian Density Function
 *      - Tells how likely a set of measurements are given car's position and noise
 *      - Each particle is independant 
 *      - Xi : Measurement i
 *      - µi : Predicted Measurements i
 *      - Σ : Covariance Measurements
 * 
 *    w = summation exp(-1/2(Xi - µi)^T * Σ^-1 (Xi - µi)) / sqrt(|2*pi*Σ|)
 * 
 *      Σ = | σxx σxy |     σxx - variance of x
 *          | σyx σyy |     σyy - variance of y
 *  
 *      Lower the values , the more the weight will be.
 *      σxy,σyx -> diagonal variances.
 */

/**
 * @brief Calculate Error
 * 
 *  error weighted = summation( w(i) * sqt(|p(i) - g|) ) / summation( w(i) ) 
 *  error best = sqt(|p(i*) - g|) , where p(i)* is the heaviest weighted particle
 * 
 *      w(i) = weight of particle i
 *      p(i) = particle i (state vector)
 *      g = Ground truth (state vector)
 * 
 *  so we can calculate error by comparing position and ground truth position
 *      
 *      Position RMSE = sqrt( ( x(p) - x(g) )^2 + ( y(p)- y(g) )^2 )
 *      Theta RMSE = sqrt( (Θ(p) - Θ(g))^2) 
 *  
 * 
 */

/**
 * @brief Tranformations and Associations
 * 
 * We will first need to transform the car's measurements from 
 * its local car coordinate system to the map's coordinate system. 
 * Next, each measurement will need to be associated with a landmark 
 * identifier, for this part we will take the closest landmark to each 
 * transformed observation. Finally, we will use this information to 
 * calculate the weight value of the particle.
 * 
 *  1. Transform: Convert from local car coordinates to map coordinates
 *  2. Associate: Take the closest landmark to each transformed observation.
 *  3. Update Weights: Calculate weight of particle
 * 
 *  TRANSFORM:
 *      - Rotate the map to match particle orientation.
 *      - The shift the orgin to the particle.
 *  
 *      | x(m) | = | cosΘ -sinΘ x(p) |  | x(c) |
 *      | y(m) | = | sinΘ  cosΘ Y(p) |  | Y(c) |
 *      |  1   | = |   0     0    1  |  |  1   |
 * 
 */

//QUIZ
void calculateTransformation(){
    double x_ref, y_ref, x_obs, y_obs, theta;
    x_ref = 4;
    y_ref = 5;
    x_obs = 0;
    y_obs = -4;
    theta = -M_PI/2;

    double x_map = (x_obs*cos(theta)) - (sin(theta)*y_obs) + (x_ref);
    double y_map = (x_obs*sin(theta)) - (cos(theta)*y_obs) + (y_ref);
    cout << x_map << ", " << (y_map) << endl;

}

/**
 * ASSOCIATION:
 * 
 * Compare location of obervations after transformation
 *  to associate with nearest neighbour.
 * 
 * UPDATEWEIGHTS: Multivariate-Gaussian probability density function
 * 
 *  Weight : P(x,y) = (1/(2*pi*σ(x)*σ(y))) * exp(-(A + B))
 *      
 *         A = ( (x - µ(x))^2 / 2σ(x)^2 )
 *         B = ( (y - µ(y))^2 / 2σ(y)^2 )
 * 
 *      µ - coordinates to nearest location
 *      σ - standard deviation
​ * 
 */

//QUIZ
void calculateParticleWeights(){
    double x_ref, y_ref, x_closest, y_closest, sigma_x, sigma_y;
    x_ref = 0;
    y_ref = 5;
    x_closest = 2;
    y_closest = 1;
    sigma_x = 0.3;
    sigma_y = 0.3;

    double coeff = 1/(2*M_PI*sigma_x*sigma_y);
    double a = (x_ref - x_closest);
    double b = (y_ref - y_closest);
    double A = ( a*a / (2*sigma_x*sigma_x));
    double B = ( b*b / (2*sigma_y*sigma_y));
    double E = exp(-1*(A+B));
    double RMSE = coeff * E;
    cout << RMSE << endl;
    //Final weight equal is all particles weight multiplied together
}