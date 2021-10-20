#include "kalman_filter.h"

namespace lipe {

  void KalmanFilter::Init(const KalmanFilterConfig* config, float prev_x, float prev_y, float current_x, float current_y){

    kalman_ = cv::KalmanFilter(4, 2, 0); 
    kalman_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    measurement_ = cv::Mat::zeros(2, 1, 5); // 2x1 matrix

    kalman_.statePost.at<float>(0) = current_x;
    kalman_.statePost.at<float>(1) = current_y;
    kalman_.statePost.at<float>(2) = current_x - prev_x;
    kalman_.statePost.at<float>(3) = current_y - prev_y;

    cv::setIdentity(kalman_.measurementMatrix); 
    cv::setIdentity(kalman_.processNoiseCov, cv::Scalar::all(config->process_noise_cov_scalar)); 
    cv::setIdentity(kalman_.measurementNoiseCov, cv::Scalar::all(config->measurement_noise_cov_scalar));
    cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(config->error_cov_post_scalar));
  }
  

  void KalmanFilter::Correct(float new_meas_x, float new_meas_y, float& prev_center_x, float& prev_center_y,
                             float& center_x, float& center_y, float& speed_x, float& speed_y){
    // save old center
    prev_center_x = center_x;
    prev_center_y = center_y;

    // new measurements
    measurement_.at<float>(0) = new_meas_x;
    measurement_.at<float>(1) = new_meas_y;

    // get new estimates
    cv::Mat estimated = kalman_.correct(measurement_);
    center_x = (estimated.at<float>(0));
    center_y = (estimated.at<float>(1));

    // update speed based on estimation
    speed_x = (center_x - prev_center_x);
    speed_y = (center_y - prev_center_y);
  }


  void KalmanFilter::Predict(float& prev_center_x, float& prev_center_y, float& center_x, float& center_y, float& speed_x, float& speed_y){
    // save old center
    prev_center_x = center_x;
    prev_center_y = center_y;

    // get new center
    cv::Mat prediction = kalman_.predict();
    center_x = (prediction.at<float>(0));
    center_y = (prediction.at<float>(1));

    // update speed based on prediction
    speed_x = (center_x - prev_center_x);
    speed_y = (center_y - prev_center_y);
  }
  
}