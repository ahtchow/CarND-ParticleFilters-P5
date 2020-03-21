/**
 * @file MarkovLocalization.cpp
 * @author Adrian Chow 
 * @brief  Markov Localization
 * @version 0.1
 * @date 2020-03-12
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Formal Definition of Variables 
 * Known:
 * @param z(1:t) represents the observation vector from time 0 to t 
 *              (range measurements, bearing, images, etc.).
 * @param u(1:t) represents the control vector from time 0 to t
 *              (yaw/pitch/roll rates and velocities).
 * @param m represents the map (grid maps, feature maps, landmarks)
 * 
 * Unknown:
 * @param x(t) represents the pose (position (x,y) + orientation \thetaθ)
 * 
 * Localization is all about estimating the pose of car (x(t)) based on 
 * previous measurements from 0 to t     
 * 
 * Thus we use the Posterior Distribution:
 *      bel(x(t)) = p( Xt | z(1:t), u(1:t))   
 * 
 * We will solve the mapping issue and localization issue:
 *      
 *  m -> Mapping feauture map (1-D)
 *      0m -  Pole (9m) - Pole(15m) - Tree(25m) - Tree(31m)
 *      m = [0,9,15,25,31]
 * 
 * z(1:t) -> Observation List (Assume can see distance)
 *         z(1:t) = {zt.....,z1} - A list of timestamps
 *         zt = [zt1,...,ztk] - A vector of distnaces
 * 
 * u(1:t) -> Control Vector
 *       u(1:t) = [ut,...,u1] - A vector of distances traveled between timestamps
 *        t-1  -------- 2m ---------> t
 *          
 * OVERALL: belief(xt) = [belief(xt = 0) ,......, belief(xt = 99)]     
 * 
 */

/**
 * @brief BAYES RULE
 * 
 * Bayes' Rule enables us to determine the conditional probability 
 * of a state given evidence P(a|b) by relating it to the 
 * conditional probability of the evidence given the state 
 * P(b|a) in the form of:
 * 
 *  P(a\b) = (P(b|a) * P(a)) / P(b) 
 * 
 *              P(B|A)     => P(B|A) * P(A)
 *      P(A)<
 *              P(B|!A)    => P(B|!A) * P(A)
 * P <         
 *              P(A|B)     => P(A|B) * P(B)
 *      P(B)<
 *              P(A|!B)    => P(A|!B) * P(B)
 * 
 * We can apply Bayes' Rule to vehicle localization by passing 
 * variables through Bayes' Rule for each time step, as our 
 * vehicle moves.
 * 
 *    For: P(a\b) = (P(b|a) * P(a)) / P(b) 
 * 
 * 1. P(location∣ observation): 
 *            This is P(a|b), the normalized probability 
 *            of a position given an observation (posterior).
 * 
 * 2. P(observation∣location): 
 *            This is P(b|a), the probability of an 
 *            observation given a position (likelihood).
 * 
 * 3. P(location): 
 *            This is P(a), the prior probability of a position
 * 
 * 4. P(observation):
 *            This is P(b), the probability of an observation
 *  
 * P(location) is determined by the motion model. The probability 
 * returned by the motion model is the product of the transition 
 * model probability (the probability of moving from 
 * x(t−1) --> x(t) and the probability of the state x(t−1).
 *
 *  As a result: 
 *  P(location|observation) = P(observation|location)* P(location)/ P(observation)
 * 
 */

/**
 * @brief BAYES FILTER
 * 
 * To build a bayes filter you must:
 * 1. Compute Bayes’ rule
 * 2. Calculate Bayes' posterior for localization
 * 3. Initialize a prior belief state
 * 4. Create a function to initialize a prior belief state
 *    given landmarks and assumptions
 * 
 * P(location|observation) = P(observation|location)* P(location)/ P(observation)
 * 
 *              P(obs|loc)     => P(obs|loc) * P(loc)
 *      P(loc)<s
 *              P(obs|!loc)    => P(obs|!loc) * P(loc)
 * P <         
 *              P(loc|obs)     => P(loc|obs) * P(obs)
 *      P(obs)<
 *              P(loc|!obs)    => P(loc|!obs) * P(obs)
 * 
 * Raw P(Posterior) = P(loc|obs) * P(obs)
 * Normalized P(Posterior) = P(loc|obs)
 * 
 * @brief Initialize Belief State
 * what values should our initial belief state take for each possible position? 
 * Let's say we have a 1D map extending from 0 to 25 meters.
 * 
 * Landmarks @ 5.0 , 10.0 & 20.0 with S.D of 1 meters
 * 
 * thus we have chances to be at:
 *      [4,5,6,9,10,11,19,20,21] out of [1-25] or 1/11
 * 
 * {0, 0, 0, 1.11E-01, 1.11E-01, 1.11E-01, 0, 0, 1.11E-01, 1.11E-01, 
 * 1.11E-01, 0, 0, 0, 0, 0, 0, 0, 1.11E-01, 1.11E-01, 1.11E-01, 0, 0, 0, 0}
 *      
 * 
 */

/***************************************************************************
 *                             CODE TIME
 * *************************************************************************/


/**
 * @brief We will create a function that initializes priors (initial belief for each
 *  position on the map) given a standard deviation of +/- 1 and the assumption that 
 *  our car is parked next to a landmark
 * 
 * We input a control of moving 1 step but our actual movement could be in the range 
 * of 1 +/- control standard deviation. The position SD is a spread of actual position.
 * 
 * For example, we may believe start at a particular location, but we could be anywhere 
 * in that location +/- our position standard deviation.
 * 
 */

#include <iostream>
#include <vector>
#include <algorithm>  

using std::vector;

// initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev);

int main() {
  // set standard deviation of position
  float position_stdev = 1.0f;

  // set map horizon distance in meters 
  int map_size = 25;

  // initialize landmarks
  vector<float> landmark_positions {5, 10, 20};

  // initialize priors
  vector<float> priors = initialize_priors(map_size, landmark_positions,
                                           position_stdev);

  // print values to stdout 
  for (int p = 0; p < priors.size(); ++p) {
    std::cout << priors[p] << std::endl;
  }

  return 0;
}

// TODO: Complete the initialize_priors function
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev) {

  // initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev

  // set all priors to 0.0
  vector<float> priors(map_size, 0.0);
  float denom = float(landmark_positions.size() * (1+position_stdev*2));
  float probability = 1/denom;
  int length = 0;

  for(int i = 0; i < map_size; i++){
      if(i == landmark_positions[length]){
          priors[i] += probability; 
          priors[i-1] += probability; 
          priors[i+1] += probability;
          length++;
      }

  return priors;
}

/**
 * @brief How much data does Z(1:t) contain?
 *  - w/ each refresh rate LiDAR sends 100,000 data points
 *  - each data point has 5 data pieces (id, range, 2 angles, reflectivity)
 *  - each data piece is 4 bytes.
 *  - Update at 10Hz per second
 *  
 *  6 hours(10Hz/s) can generate 432 GB of data. 
 * 
 *  Two problems if we want to estimate bel(x(t)) directly
 *  1. A lot of data
 *  2. Amount of data increases over time
 */

/**
 * @brief Definition fo Localization Posterior: P(loc|observation)
 * 
 *  bel(x(t)) = p( x(t) | z(1:t), u(1:t), m)   
 *  
 * We do not want to store all observations, but rather update the 
 * state estimator recursively w/ new observation.
 * 
 * WE CALL THIS; MARKOV LOCALIZATION
 *    achieved using:
 *      1. Bayes Rule
 *      2. Law of Total Probability
 *      3. Markov Assumption
 *  
 * Also we must split z(1:t) to z(t), z(1:t-1) as current obs and prev obs
 *  in bel(x(t)) = p( x(t) | z(t), z(1:t-1), u(1:t) , m) 
 * 
 * BAYES RULE:
 *      bel(x(t)|z(t)) = p( z(t) | x(t), z(1:t-1), u(1:t) , m)  * 
 *                      p( x(t) | z(1:t-1), u(1:t) , m) /
 *                      p( z(t) | z(1:t-1), u(1:t) , m)
 * 
 *       P(loc) = P(x(t)) , P(obs) = P(z(t)) 
 * 
 * LAW OF TOTAL PROBABILITY: P(A) = ∑ P(B|A) * P(B)
 * 
 *  p( x(t) | z(1:t-1), u(1:t) , m) 
 *      = ∑ p( x(t) | x(t-1), z(1:t-1), u(1:t) , m) * 
 *          p( x(t-1) | z(1:t-1), u(1:t) , m)  
 * 
 *  
 * MARKOV ASSUMPTION:
 *  1. Since we (hypothetically) know in which state the system is at time step t-1, 
 * the past observations z_{1:t-1} and controls u{1:t-1} would not provide new info.
 * 
 *      Thus  p( x(t) | x(t-1), z(1:t-1), u(1:t) , m) can be simplified to 
 *         p( x(t) | x(t-1), u(t), m) (TRANSITION/SYSTEM MODEL)
 *    
 *      or simply: P(x(t)|x(1:t-1)) = P(x(t)|x(t-1))
 * 
 * 2.  Since u(t) is “in the future” with reference to x{t-1}, u(t) 
 *     does not tell us much about x{t-1}.
 * 
 *    Thus p (x(t-1)| z(1:t-1) , u(1:t), m ) can be simplified to 
 *        p (x(t-1)| z(1:t-1) , u(1:t-1), m )
 * 
 * OVERALL:
 *    P(x(t)|z(1:t-1), u(1:t), m) = 
 *      ∫ p( x(t) | x(t-1), u(t), m) *  p (x(t-1)| z(1:t-1) , u(1:t-1), m ) dx(t-1)
 *    = ∫ p( x(t) | x(t-1), u(t), m) *  bel(x(t-1)) dx(t-1) (Recursive Case)
 *    = ∑ p( x(t) | x(t-1), u(t), m) *  bel(x(t-1)) (Discrete Case)
 *  
 *  After applying the Markov Assumption, the term p(x{t-1} | z{1:t-1}, u{1:t-1}, m)
 *  describes exactly the belief at x{t-1}. This means we achieved a 
 *  recursive structure!
 * 
 *  The prediction of x(t) should simply reply on the previous belief 
 *  bel(x(t-1)) , the current control state u(t) and the map m.
 * 
 *  */

