/**
 * The blob class for "lipe". 
 * 
 * It runs the steps of the algo in sequence.
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Károly Harsányi (CC-AD/EAU-BP)
 */
 

#ifndef LIPE_BLOB_H_
#define	LIPE_BLOB_H_

#include "GlobalHeader.h" //TODO: rename global header -> lipe_types.h
#include "kalman_filter.h"

namespace lipe{
  
  // for blobs classified as objects
  enum ObjectStatus{
    PLACEHOLDER = 0, 
    IN_DETECTION = 1,
    IN_TRACKING = 2,
  };

class Blob {
  public:
    // Status
    ObjectStatus status_;
 
    // data members for detection
    int num_points_;
    int num_cells_;

    int min_i_cell_idx_; // top cell
    int min_j_cell_idx_; // left cell
    int max_i_cell_idx_; // bottom cell
    int max_j_cell_idx_; // right cell 

    float min_x_, max_x_;
    float min_y_, max_y_;
    float min_z_, max_z_; 

    bool in_ego_circle_; // true, if any part of the blob is in close proximity of the ego vehicle

    float prev_center_x_;
    float prev_center_y_;
    float center_x_;
    float center_y_;
    float speed_x_;
    float speed_y_;

    pcl::RGB color_;    
    PointClasses label_;
  
    int ID_;
    bool is_active_; // true, if its found on the current frame
    uint last_seen_; // 0 if it is active, incremented every time it is inactive
    uint continously_seen_; // incemeneted if its active. else it is set to zero 
    uint max_continously_seen_; // incemeneted if its active. else it is set to zero 
    bool visualized_;
    
    Blob* tracked_version_; 
    
    // default constructor
    Blob(); 

    /**
     * @brief initializes data members with default values
     */ 
    void Init(); // initializes data members
    
    // KalmanFilter
    bool kalman_active_;
    const KalmanFilterConfig* kalman_config_;
    KalmanFilter kalman_;
    
    /**
    * @brief initalizes the kalman filter. it is called after the blob is observed for 2 frames
    */
    void InitKalman();
    
    /**
    * @brief Calls the kalman filter prediction, which updates the center
    */
    void PredictKalman();
    
    /**
    * @brief calls the kalman filter correction on the new measurements
    * @param new_meas_x: measurement for the x coordinate of the center
    * @param new_meas_y: measurement for the y coordinate of the center
    */
    void CorrectKalman(float new_meas_x, float new_meas_y);
};

}
#endif // LIPE_BLOB_H_
