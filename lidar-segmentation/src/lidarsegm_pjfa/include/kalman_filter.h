#ifndef KALMAN_FILTER_H_
#define	KALMAN_FILTER_H_

#include "GlobalHeader.h"
#include "config.h"

namespace lipe{
  
class KalmanFilter{
  public:
 
    KalmanFilterConfig* config_;
      
    cv::KalmanFilter kalman_;
    cv::Mat measurement_;
  
    // TODO: KF config in config.h, use it to build the KF class
    // TODO: examine more complex Kalman filter setups and versions 
    //       (i.e. predict for longer timeperiods, extended Kalman filters etc.)
    
    /** 
    * @brief initializes Kalman filter.
    * 
    * sets measurements and covariance matrices to idenity * some scalar
    *
    * @params prev_{x,y} and current_{x,y} are the coordinates of the center points 1 step before the activation of the kf, and now
    * @param process_noise_cov_scalar is the scalar for the process noise covariance matrix
    * @param measurement_noise_cov_scalar is the scalar for the measurement noise covariance matrix
    * @param error_cov_post_scalar is scalar for the priori error estimate covariance matrix
    */ 
    void Init(const KalmanFilterConfig* config, float prev_x, float prev_y, float current_x, float current_y);
    
    /** 
    * @brief use Kalman filter for prediction 
    * 
    * stores the current x and y center in the prev_cx_ and prev_cy_ data members
    * predicits center_x_ and center_y_, and speed_x_ and speed_y_ 
    */                  
    void Predict(float& prev_center_x, float& prev_center_y, float& center_x, float& center_y, float& speed_x, float& speed_y);
    
    /** 
    * @brief corrects Kalman filter based on new incoming measurements
    * 
    * stores the current x and y center in the prev_cx_ and prev_cy_ data members
    * feeds the new incoming measurements to the Kalman filter
    * updates center_x_ and center_y_, and speed_x_ and speed_y_ 
    *
    * @param new_cx_measurement is the new measurement for the x coordinate of the objects center
    * @param new_cy_measurement is the new measurement for the y coordinate of the objects center
    */
    void Correct(float new_meas_x, float new_meas_y, float& prev_center_x, float& prev_center_y,
                 float& center_x, float& center_y, float& speed_x, float& speed_y);
};
  
}
#endif // KALMAN_FILTER_H_