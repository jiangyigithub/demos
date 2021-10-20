/**
 * The tracker class for "lipe". 
 * 
 * It runs the steps of the algo in sequence.
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Károly Harsányi (CC-AD/EAU-BP)
 */

#ifndef LIPE_TRACKER_H_
#define	LIPE_TRACKER_H_

#include "blob.h"
#include "logger.h"
#include "../lib/hungarian_method/include/HungarianMethod.h"

namespace lipe{
  class Tracker{
  private:
    
    int num_tracked_objects_; // number of tracked objects
    int frame_counter_; // keeps track of number of frames in the tracker's lifecycle
    int current_rolling_ID_; // rolling ID for the class. each new object ID is assigned based on this
    const TrackerConfig* config_; // config params (see config.h)
    Blob* tracked_objects_; // keeps track of the tracked objects
  
   /**
    * @brief when a new Blob enters the tracking this function updates its tracker related params
    */
    void SetNewBlobParams(Blob& b);
  
   /**
    * Calculates the similarity between the input blobs, based on the IoU of their bounding boxes and the distance between their center
    * @params b1 and b2 Blobs
    */
    float GetSimilarity(const Blob& b1, const Blob& b2);
    
   /**
    * Calculates the IoU of the two input blobs
    * @params b1 and b2 Blobs
    */
    float GetIoU(const Blob& b1, const Blob& b2);
    
    
   /**
    * Calculates the distance between the input blobs' center
    * @params b1 and b2 Blobs
    */
    float GetCenterDistance(const Blob& b1, const Blob& b2);
 
   /**
    * Assigns float value to the blob based on its lifetime. A fresh blob gets a lower value
    * @params b1 Blob
    */
    float GetTimeValue(const Blob& b);
    
   /**
    * Initalizes the tracking.
    * This function is called when tracking starts, and when there are no previous objects to track
    * @params current_objects: array, containing the currently detected objects by the blobdetector
    * @params current_objects_length is the length of the current_objects array
    */
    void InitTracking(Blob** current_objects, int current_objects_length);
    
    /**
     * @brief removes the outdated blobs from the tracked_objects_ array based on the config_.forget time param
     */
    void RemoveOutdatedBlobs();
    
    /**
     * @brief runs through the tracked_objects_ array and predicts the position of the blobs on the next frame, using their kalman filters
     */
    void MakePredictedMovements();
    
    /**
     * @brief runs through the tracked_objects_ array, and compares the blobs between these objects and the currently detected object list of the detector
     * uses based on their pairwise similarity and the Hungarian method, it assigns the corresponding blobs and refreshes the old version with the new ones
     */
    void RefreshTrackedBlobs(Blob** current_objects, int current_objects_length, int* assignments);
  
   /**
    * @brief Based on our similarity measure, it determines a similarity value between the incoming blobs from the detector and the tracked blobs
    * @returns an array containing these similarity values
    */
    int* GetAssignments(Blob** current_objects, int current_objects_length); // use Hungarian method to get some assignments
    
   /**
    * @brief goes through the tracked blob list, if there are valid blobs without IDs assingned,
    * it assigns an new ID and increments the class's rolling ID  
    */ 
    void AssignBlobIDs();
    
    /**
     * @brief if the blobs are assigned to each other based on their similarity, this functions updates the old blob based on the new measurements
     * @param old_blob: blob in the tracked objects list
     * @param new_blob: the new version of the old_blob based on the detection step and the assigmments
     */
    void UpdateBlob(Blob* old_blob, Blob* new_blob);
      
  public: 
    // simple constructor. zeros the rolling ID, constructs the tracked_objects_ etc.
    Tracker(const TrackerConfig& config);
    
    // runs the main algorithm. 
    void Run(Blob** current_objects, int current_objects_length); // ~ input two blob vectors. tracked and current. does changes to the tracked blobs 
    
    // these functions return the traced objects. these are called by the velodyne_segmenter_algo containing this calss
    lipe::Blob* GetTrackedObjects();
    int GetMaxTrackedLen();
    
    // returns the number of tracked objects
    int GetNumTracked();
    
  };  
}
#endif // LIPE_TRACKER_H_