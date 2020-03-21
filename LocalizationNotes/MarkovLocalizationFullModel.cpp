#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

using std::vector;
using std::cout;
using std::endl;

class Helpers {
public:

	//definition of one over square root of 2*pi:
	float STATIC_ONE_OVER_SQRT_2PI = 1/sqrt(2*M_PI) ;
	float ONE_OVER_SQRT_2PI = 1/sqrt(2*M_PI) ;

	/*****************************************************************************
	 * normpdf(X,mu,sigma) computes the probability function at values x using the
	 * normal distribution with mean mu and standard deviation std. x, mue and 
	 * sigma must be scalar! The parameter std must be positive. 
	 * The normal pdf is y=f(x;mu,std)= 1/(std*sqrt(2pi)) e[ -(x−mu)^2 / 2*std^2 ]
	*****************************************************************************/
	float normpdf(float x, float mu, float std) {
	    return (STATIC_ONE_OVER_SQRT_2PI/std)*exp(-0.5*pow((x-mu)/std,2));
	}

	//static function to normalize a vector:
	std::vector<float> normalize_vector(std::vector<float> inputVector){

		//declare sum:
		float sum = 0.0f;

		//declare and resize output vector:
		std::vector<float> outputVector ;
		outputVector.resize(inputVector.size());

		//estimate the sum:
		for (unsigned int i = 0; i < inputVector.size(); ++i) {
			sum += inputVector[i];
		}

		//normalize with sum:
		for (unsigned int i = 0; i < inputVector.size(); ++i) {
			outputVector[i] = inputVector[i]/sum;
		}

		//return normalized vector:
		return outputVector;
	}
};
// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions, 
                        vector<float> observations, vector<float> pseudo_ranges, 
                        float distance_max, float observation_stdev) {
  // initialize observation probability
  float distance_prob = 1.0f;

  // run over current observation vector
  for (int z=0; z< observations.size(); ++z) {
    // define min distance
    float pseudo_range_min;
        
    // check, if distance vector exists
    if (pseudo_ranges.size() > 0) {
      // set min distance
      pseudo_range_min = pseudo_ranges[0];
      // remove this entry from pseudo_ranges-vector
      pseudo_ranges.erase(pseudo_ranges.begin());
    } else {  // no or negative distances: set min distance to a large number
        pseudo_range_min = std::numeric_limits<const float>::infinity();
    }

    // estimate the probability for observation model, this is our likelihood 
    Helpers help_me;
    distance_prob *= help_me.normpdf(observations[z], pseudo_range_min,
                                      observation_stdev);
  }

  return distance_prob;
}

vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position) {
  // define pseudo observation vector
  vector<float> pseudo_ranges;
            
  // loop over number of landmarks and estimate pseudo ranges
  for (int l=0; l< landmark_positions.size(); ++l) {
    // estimate pseudo range for each single landmark 
    // and the current state position pose_i:
    float range_l = landmark_positions[l] - pseudo_position;

    // check if distances are positive: 
    if (range_l > 0.0f) {
      pseudo_ranges.push_back(range_l);
    }
  }

  // sort pseudo range vector
  sort(pseudo_ranges.begin(), pseudo_ranges.end());

  return pseudo_ranges;
}

// motion model: calculates prob of being at an estimated position at time t
float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev) {
  // initialize probability
  float position_prob = 0.0f;

  // loop over state space for all possible positions x (convolution):
  for (float j=0; j< map_size; ++j) {
    float next_pseudo_position = j;
    // distance from i to j
    float distance_ij = pseudo_position-next_pseudo_position;

    // transition probabilities:
    Helpers help_me;
    float transition_prob = help_me.normpdf(distance_ij, movement, 
                                             control_stdev);
    // estimate probability for the motion model, this is our prior
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
  // set standard deviation of control
  float control_stdev = 1.0f;

  // set standard deviation of position
  float position_stdev = 1.0f;

  // meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  // set observation standard deviation
  float observation_stdev = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // set distance max
  float distance_max = map_size;

  // define landmarks
  vector<float> landmark_positions {3, 9, 14, 23};

  // define observations vector, each inner vector represents a set 
  //   of observations for a time step
  vector<vector<float> > sensor_obs {{1,7,12,21}, {0,6,11,20}, {5,10,19},
                                     {4,9,18}, {3,8,17}, {2,7,16}, {1,6,15}, 
                                     {0,5,14}, {4,13}, {3,12}, {2,11}, {1,10},
                                     {0,9}, {8}, {7}, {6}, {5}, {4}, {3}, {2},
                                     {1}, {0}, {}, {}, {}};

  /**
   * TODO: initialize priors
   */
  vector<float> priors = initialize_priors(map_size,landmark_positions, position_stdev);

  cout << "-----------PRIORS INIT--------------" << endl;
  for (int p = 0; p < priors.size(); ++p){
   cout << priors[p] << endl;
  }  
    
  // initialize posteriors
  vector<float> posteriors(map_size, 0.0);

  // specify time steps
  int time_steps = sensor_obs.size();
    
  // declare observations vector
  vector<float> observations;
    
  // cycle through time steps
  for (int t = 0; t < time_steps; ++t) {

    cout << "---------------TIME STEP---------------" << endl;
    cout << "t = " << t << endl;
    cout << "-----Motion----------OBS----------------PRODUCT--" << endl;

    if (!sensor_obs[t].empty()) {
      observations = sensor_obs[t]; 
    } else {
      observations = {float(distance_max)};
    }

    // step through each pseudo position x (i)
    for (unsigned int i = 0; i < map_size; ++i) {
      float pseudo_position = float(i);

      /**
       * Get the motion model probability for each x position
       */

      float motion_prob = motion_model(
          pseudo_position, 
          movement_per_timestep, 
          priors, 
          map_size, 
          control_stdev);

      /**
       * Get pseudo ranges
       */
      
      vector <float> pseudo_ranges = pseudo_range_estimator(
          landmark_positions,
          pseudo_position);

      /**
       * Get observation probability
       */

      float observation_prob = observation_model(
          landmark_positions,
          observations,
          pseudo_ranges,
          distance_max,
          observation_stdev);

      /**
       * Calculate the ith posterior and pass to posteriors vector P(loc|obs).
       */
      
      posteriors[i] = motion_prob * observation_prob;

      // UNCOMMENT TO SEE THIS STEP OF THE FILTER
      cout << motion_prob << "\t" << observation_prob << "\t" 
          << "\t"  << motion_prob * observation_prob << endl;   
    } 
        
    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    cout << "----------RAW---------------" << endl;
    for (int p = 0; p < posteriors.size(); ++p) {
     cout << posteriors[p] << endl;
    }

    /**
     * Normalize posteriors (see helpers.h for a helper function)
     */

    Helpers help_me;
    posteriors = help_me.normalize_vector(posteriors);
    
    cout << posteriors[t] <<  "\t" << priors[t] << endl;
    cout << "----------NORMALIZED---------------" << endl;

    /**
     * Update priors
     * 
     */

    priors = posteriors;

    // print posteriors vectors to stdout
    for (int p = 0; p < posteriors.size(); ++p) {
            cout << posteriors[p] << endl;  
    } 
  }
  
  return 0;
}

