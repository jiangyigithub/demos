/**
 * The detector class for "lipe". 
 * 
 * It runs the steps of the algo in sequence.
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Károly Harsányi (CC-AD/EAU-BP)
 */

#ifndef LIPE_BLOBDETECTOR_H_
#define	LIPE_BLOBDETECTOR_H_

#include "blob.h"
#include "grid.h"
#include "logger.h"


namespace lipe {


class BlobDetector {
  private:
    const DetectorConfig* config_;
    const KalmanFilterConfig* kalman_config_;
    Grid* grid_;
    int** visited_;
       
   /**
    * @brief resets the blob_list_, the object_list_, the other_list_ and the visited_ data members
    */
    void ResetBlobList(); // empties out the bloblist and refilles it with tmp blobs
    
   /**
    * @brief loads blob into the object list and increments the list length
    * @param blob is a pointer to a blob in the bloblist.
    * 
    * if the object list is full, the function ommits the call, otherwise the blob is placed in the object list and labeled as OBJECT
    *
    */
    void SetToObject(Blob* blob);
    
    
   /**
    * @brief loads blob into the other list and increments the list length
    * @param blob is a pointer to a blob in the bloblist.
    * 
    * if the other list is full, the function ommits the call, otherwise the blob is placed in the other list and labeled as OTHER
    *
    */
    void SetToOther(Blob* blob);
    
  public:
    int current_object_list_len_;
    int current_blob_list_len_;
    int current_other_list_len_;
    Blob** object_list_; // blobs classified as objects
    Blob* blob_list_; // every blob detected
    Blob** other_list_; // blobs classified as others
    
   /**
    * @brief initalizes the detector
    * 
    * constructcs the blob lists and fills them with default values
    * 
    * @param config contains the params that are used during the blob detection and classification, 
    * as well as information regarding the predefined max length of the blob lists
    * @param grid_config contains general info about the grid's size etc. 
    */
    BlobDetector( const DetectorConfig& config, const KalmanFilterConfig& kalman_config, Grid& grid);
    
   /**
    * @brief resposible for detecting blob in the grid
    * 
    * calls ResetBlobList() to clear the data members,
    * runs through the grid, and looks for cells classified as NOT_CLASSIFIED (not road). When finding a cell like this,
    * it initializies a new blob, and starts a BFS from the cell to explore its surroundings and look for neighboring,
    * unvisited cells with the same NOT_CLASSIFIED class. during the BFS it updates the Blobs data members accordingly.
    * When the function is finished, the blob_list_ is filled with unclassified blobs. Later, the ClassifiyBlobs function is 
    * responsible for the classification (or ideally the PointNet).
    */
    void FillBlobList();
    
   /**
    * @brief classfies the blobs into 'objects' and 'others' based on heuristics from the config data 
    */
    void ClassifiyBlobs(); // classification based on heuristics
    
    Blob** GetObjectList()  { return object_list_; }   
    int GetObjectListLength() { return current_object_list_len_;}
};

}

#endif // LIPE_BLOBDETECTOR_H_