/**
 * 
 * @brief Implementation details for motion model
 *  P(x(t)|z(1:t-1), u(1:t), m) =  
 *  ∑ p( x(t) | x(t-1), u(t), m) *  bel(x(t-1)) (Discrete Case)
 * 
 *  1. Assumption: The car is parked at a tree/street lamp
 *      plus or minus 1 meters
 * 
 *  2. Transition model is only controlled by x(t-1) and u(t)
 *      and independant from the map m. p(x(t)|x(t-1), u(t)).
 * 
 *  3. Modelled as 1-D normal distribution:
 *        p(x(t)|x(t-1), u(t)) = N(x(t) - x(t-1); u(t), VARIANCE)
 * 
 * 
 *  Noise:
 *       - Our initial belief is uniform and filled w/ max confusion.
 *       - If we assume the car is parked closely to a tree we can be more confident.
 *  
 *  We will use normal distribution and a probability density function to
 *  predict the probability of the next step.      
 * 
 *  'i'th Motion Model Probability:
 *      ∑ p( x(t) | x(t-1)i, u(t), m) *  bel(x(t-1)i) 
 *      
 */

 /**
  *  @brief For each x(t) :
  * 
  *     1. Calculate the transition probability for each potential
  *         value x(t-1)
  * 
  *     2. Calculate the discrete motion model probability by multiplying
  *         the transition model probability by the belief state (prior)
  *         for x(t-1)
  * 
  *     3. Return total probability(sum) of each discrete probability 
  *         that should make up a normal distribution
  * 
  * 
  */

#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

class Helpers {
 public:
  // definition of one over square root of 2*pi:
  float STATIC_ONE_OVER_SQRT_2PI = 1/sqrt(2*M_PI);

  /**
   * normpdf(X,mu,sigma) computes the probability function at values x using the
   * normal distribution with mean mu and standard deviation std. x, mu and 
   * sigma must be scalar! The parameter std must be positive. 
   * The normal pdf is y=f(x,mu,std)= 1/(std*sqrt(2pi)) e[ -(x−mu)^2 / 2*std^2 ]
   */
  float normpdf(float x, float mu, float std) {
    return (STATIC_ONE_OVER_SQRT_2PI/std)*exp(-0.5*pow((x-mu)/std,2));
  }

};
//Implement the motion model: calculates prob of being at 
// an estimated position at time t
float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev) {
  // initialize probability
  float position_prob = 0.0f;
  
  for(int j = 0; j < map_size; ++j){
      float next_pseudo_position = j;
      float distance_a_to_b = pseudo_position - next_pseudo_position;
      Helpers helper;
      float transition_prob = helper.normpdf(distance_a_to_b,movement, control_stdev);
      position_prob += transition_prob*priors[j];
  }
  return position_prob;
}

// initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                     float position_stdev) {

  // set all priors to 0.0
  vector<float> priors(map_size, 0.0);

  // set each landmark positon +/-1 to 1.0/9.0 (9 possible postions)
  float norm_term = landmark_positions.size() * (position_stdev * 2 + 1);
  for (int i=0; i < landmark_positions.size(); ++i) {
    for (float j=1; j <= position_stdev; ++j) {
      priors.at(int(j+landmark_positions[i]+map_size)%map_size) += 1.0/norm_term;
      priors.at(int(-j+landmark_positions[i]+map_size)%map_size) += 1.0/norm_term;
    }
    priors.at(landmark_positions[i]) += 1.0/norm_term;
  }
  
  return priors;
}

int main() {
  // set standard deviation of control:
  float control_stdev = 1.0f;

  // set standard deviation of position:
  float position_stdev = 1.0f;

  // meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // initialize landmarks
  vector<float> landmark_positions{5, 10, 20};
    
  // initialize priors
  vector<float> priors = initialize_priors(map_size, landmark_positions,
                                           position_stdev);
    
  // step through each pseudo position x (i)    
  for (float i = 0; i < map_size; ++i) {
    float pseudo_position = i;

    // get the motion model probability for each x position
    float motion_prob = motion_model(pseudo_position, movement_per_timestep,
                                     priors, map_size, control_stdev);
        
    // print to stdout
    std::cout << pseudo_position << "\t" << motion_prob << std::endl;
  }    

  return 0;
}

