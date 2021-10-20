/*
 * kitti_parser.h
 *
 *  Created on: Oct 1, 2018
 *      Author: kvv2bp
 */

#ifndef INCLUDE_KITTI_PARSER_H_
#define INCLUDE_KITTI_PARSER_H_

#include<GlobalHeader.h>
#include "../../include/segmenter.h"

struct tBox{
  std::string type;
  double x1;
  double x2;
  double y1;
  double y2;
  double orient;
  tBox (std::string type, double x1, double x2, double y1, double y2, double orient) :
    type(type), x1(x1), y1(y1), x2(x2), y2(y2), orient(orient) {}
};

struct tDetection
{
  tBox box;
  double thresh;
  double ry;
  double t1, t2, t3;
  double h, w, l;
  Eigen::MatrixXf* bb3d;
  tDetection():
    box(tBox("invalid", -1, -1, -1, -1, -10)), thresh(-1000){}
  tDetection(tBox box, double thresh):
    box(box), thresh(thresh) {}
  tDetection(std::string type, double x1, double x2, double y1, double y2, double orient, double thresh) :
    box(tBox(type, x1, x2, y1, y2, orient)), thresh(thresh){}
};

class KittiParser
{
  private:
    Cloud cloud;
    std::vector<tDetection> detections;
    lidarsegm_pjfa::segmenterBasic segm;

  public:
    KittiParser(std::string file_path);
    Cloud* getCloud(){return &cloud;}
    void loadDetectionsFromTxt(std::string file_path, Eigen::MatrixXf* velo2cam, Eigen::MatrixXf* r0);
    void saveToPcdFile(std::string file_path);
    static Eigen::MatrixXf* loadCalibrationFile(std::string filePath, const char* calibType,  int rowNum, int columN);
    void calcBBforDetection(tDetection* det, Eigen::MatrixXf* velo2cam, Eigen::MatrixXf* r0);
};




#endif /* INCLUDE_KITTI_PARSER_H_ */
