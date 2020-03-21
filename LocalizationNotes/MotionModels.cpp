/**
 * @file MotionModels.cpp
 * @author Adrian Chow
 * @brief 
 * @version 0.1
 * @date 2020-03-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Motion Models
 * 
 * Assumptions:
 *  - Motion is only 2-D (Modelled after Bicycle)
 *  - Two front wheels are centered as one, same as back wheels
 *  
 *  Uses heading angle and speed to measure velocity.
 *  
 *      |---------|  (As a bicycle) 
 * 
 */

/**
 * @brief Yaw Rate and Velocity
 * 
 *  θ = heading
 *  θ. = YAW RATE (change in angle)
 *  v = velocity
 * 
 *  if θ. = 0
 *  xf = xo + v(Δt)cos(θ)
 *  yf = yo + v(Δt)sin(θ)
 * 
 *  if θ. != 0
 *  xf = xo + (v/θ.)(sin(θo + θ.(dt)) - sin(θo))
 *  yf = yo + (v/θ.)(-cos(θo + θ.(dt)) + cos(θo))
 *  θf = θo + θ.(Δt)
 *   
 *  Frame of Reference: Localization vs Sensor Fusion?
 *      Localization
 *          1. Measurements are relative to a map , not the car.
 *          2. Map landmarks are used
 *          3. Described in vechile or map coordinates. (Not only vechile)
 *     
 *  Do we need more than just YAW. (I.e roll and picth)?
 *      Depends on the road''s curvature and steepness.
 * 
 */

/**
 * @brief Odometry
 *  
 *  Type of sensor that measures how many times the wheels have completed a rotation
 *  - Thus we can calculated the distance travelled.
 *          
 *       xf = xo + (num of turns)(cos(0)(circumfrence)  
 *       yf = yo + (num of turns)(sin(0)(circumfrence)  
 * 
 *      Error: Slick wet roads (slip , thus < rotation) and 
 *              lots a bumpy roads ( Does not go straight )
 * 
 *      Unaffected: Flat pavement and turns.
 * 
 */
