/**
 * @brief Now for BAYES RULE
 * 
 * We can use Markovs Assumptions again to simplify the BAYES rules
 * 
 *      p(loc|obs) = p(obs|loc) * p(loc) / p(obs)
 * 
 *      bel(x(t)|z(t)) = p( z(t) | x(t), z(1:t-1), u(1:t) , m)  * 
 *                      p( x(t) | z(1:t-1), u(1:t) , m) /
 *                      p( z(t) | z(1:t-1), u(1:t) , m)
 * 
 *      simplified to:
 *      
 *      normalized observation model =
 *          normalized( p( z(t) | x(t), z(1:t-1), u(1:t) , m)  * 
 *                      p( x(t) | z(1:t-1), u(1:t) , m) )
 *          = p(z(t)| x(t),  z(1:t-1), u(1:t) , m)
 * 
 *      Using Markovs Assumption: p(z(t)| x(t),  z(1:t-1), u(1:t) , m)
 * 
 *           observation model  = p(z(t)| x(t), m)
 *           location/motion model = p( x(t) | z(1:t-1), u(1:t) , m) 
 * 
 *      bel(x(t)|z(t)) =  p(z(t)| x(t), m) *  p( x(t) | z(1:t-1), u(1:t) , m)  /
 *                          p( z(t) | z(1:t-1), u(1:t) , m)
 */

/**
 * @brief Implementation of the Observation Model
 * 
 *  1. Measure the range closest static object
 *  2. Range noise of Gaussian is again VARIANCE = 1
 *  3. Range spectrum is 0-100m 
 *  4. Only foward measurements are included
 *  5. x(t) and m are only used to estimate pseudo range z(t)*
 * 
 *  z(t)* is the mean of pseudo ranges of z(t)
 * 
 *  SO the observation model is defined by:
 *      
 *    p(z(t)|x(t)) ~ N (z(k:t), z(t)* , VARIANCE) 
 * 
 */

/**
 * @brief TO SUMMARIZE
 * 
 *  Observation Model: p(z(t)|x(t), m) = p(obs|loc)
 *  Motion Model: ∫ p( x(t) | x(t-1), u(t), m) *  bel(x(t-1)) dx(t-1)  
 *               = p(loc) 
 * 
 *  Combine two to become BAYES FILTER (MARKOV LOCALIZATION)
 *  
 *  bel(x(t)) = p(x(t)| z(t)) = 
 *     
 *    ~Normalized[(p(z(t))|x(t),m)] *
 *               ∫ p( x(t) | x(t-1), u(t), m) *  bel(x(t-1)) dx(t-1) 
 *  
 * 
 */


#include <algorithm>
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

/**
 * @brief For each landmark position
 *      1. Determine the distance between each pseudo position x
 *          and each landmark position
 *      2. If the distance is positive(landmark is foward of the 
 *          pseudo position) push the distance to pseudo range vector
 *      3. Sort the pseudo range vector in ascending order
 *      4. Return the pseudo range vector
 * 
 */

vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position) {
  // define pseudo observation vector
  vector<float> pseudo_ranges;
            
  // loop over number of landmarks and estimate pseudo ranges
  for(float t = 0; t < landmark_positions.size(); t++){
      float dist_btwn = landmark_positions[t] - pseudo_position;
      if(dist_btwn > 0)
        pseudo_ranges.push_back(dist_btwn);
  } 
  // sort pseudo range vector
  sort(pseudo_ranges.begin(), pseudo_ranges.end());
    
  return pseudo_ranges;
}

/**
 * @brief For each observation
 *  1. Determine if a pseudo range vector exists for the
 *      current pseudo position x 
 *  2. If the vector exists , extract and store the minimum dist
 *      , element 0 of the sorted vector, and remove that element.
 *  3.  If the pseudo range vector does not exist ,pass the maximum distance
 *      to norm_pdf
 *  4. Use norm_pdf to determine the observation model probability
 *  5. Return the probability.
 */

float observation_model(vector<float> landmark_positions, 
                        vector<float> observations, vector<float> pseudo_ranges, 
                        float distance_max, float observation_stdev) {
  float distance_prob = 1.0f;
  float min_distance;
  for(int z = 0; z <observations.size(); z++){
    if(!pseudo_ranges.empty()){
        min_distance = pseudo_ranges[0];
        pseudo_ranges.erase(pseudo_ranges.begin());

        }
    else{
        min_distance = distance_max;
    }
    Helpers help;
    distance_prob *= help.normpdf(observations[z], min_distance, observation_stdev);
  }
  return distance_prob;
}
int main() {  
  // set observation standard deviation:
  float observation_stdev = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // set distance max
  float distance_max = map_size;

  // define landmarks
  vector<float> landmark_positions {5, 10, 12, 20};

  // define observations
  vector<float> observations {5.5, 13, 15};

  // step through each pseudo position x (i)
  for (int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);

    // get pseudo ranges
    vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, 
                                                         pseudo_position);

    //get observation probability
    float observation_prob = observation_model(landmark_positions, observations, 
                                               pseudo_ranges, distance_max, 
                                               observation_stdev);
    //print to stdout
    std::cout << observation_prob << std::endl; 
  }      

  return 0;
}