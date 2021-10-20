/*
 * kitti_parser.cpp
 *
 *  Created on: Oct 1, 2018
 *      Author: kvv2bp
 */
#include <stdio.h>
#include <Eigen/Core>
#include <kitti_parser.h>

void inverseRigidTrans(Eigen::MatrixXf* matrix);
bool isPointInBB(Eigen::MatrixXf* bb, Eigen::Vector3f* point);

KittiParser::KittiParser(std::string file_path)
{
  cloud.clear();
  std::fstream input(file_path.c_str(), std::ios::in | std::ios::binary);
  if(input.good())
  {
    float* dummy = new float;
    input.seekg(0, std::ios::beg);
    while(input.good() && !input.eof())
    {
      PCLMyPointType point;
      input.read((char*)&point.x, 3 * sizeof(float));
      input.read((char*)dummy, sizeof(float));
      point.r = 255;
      point.g = 255;
      point.b = 255;
      cloud.push_back(point);
    }
    input.close();
    delete dummy;
  }
}

Eigen::MatrixXf* KittiParser::loadCalibrationFile(std::string filePath, const char* calibType, int rowNum, int columNum)
{
  FILE *fp = fopen((filePath.c_str()), "r");
  float input [rowNum * columNum];
  Eigen::MatrixXf* calib = new Eigen::MatrixXf(rowNum * columNum, 1);
  while(!feof(fp))
  {
      char str[255];
      fscanf(fp, "%s %f %f %f %f %f %f %f %f %f %f %f %f",
             str, &input[0], &input[1], &input[2],
             &input[3], &input[4], &input[5], &input[6],
             &input[7], &input[8], &input[9], &input[10],
             &input[11]);
      if(strcmp(str, calibType) == 0) break;
  }
  for(int i = 0; i < rowNum * columNum; ++i)
  {
      (*calib)(i, 0) = input[i];
  }
  calib->resize(rowNum, columNum);
  calib->transposeInPlace();
  return calib;
}

void KittiParser::loadDetectionsFromTxt(std::string file_path, Eigen::MatrixXf* velo2cam, Eigen::MatrixXf* r0)
{
  detections.clear();
  FILE *fp = fopen(file_path.c_str(), "r");
  while(!feof(fp))
  {
    tDetection d;
    double trash;
    char str[255];
    fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        str, &trash, &trash, &d.box.orient, &d.box.x1, &d.box.y1,
        &d.box.x2, &d.box.y2, &d.h, &d.w, &d.l, &d.t1, &d.t2, &d.t3,
        &d.ry, &d.thresh);
    d.box.type = str;
    if(d.t1 != -1000 && d.t2 != -1000 && d.t3 != -1000 && d.h > 0 && d.w > 0 && d.l > 0 && d.box.orient != -10 && d.box.type != "DontCare")
    {
      calcBBforDetection(&d, velo2cam, r0);
      detections.push_back(d);
      std::cout<<"type " << d.box.type  << " x : "<<d.t1 << " y: "<<d.t2<<" z: "<<d.t3<<std::endl;
    }
  }
  fclose(fp);
  for(auto point = cloud.points.begin(); point < cloud.points.end(); point++)
  {
    for(auto detection = detections.begin(); detection < detections.end(); detection++)
    {
      Eigen::Vector3f pointCoord(point->x, point->y, point->z);

      if(isPointInBB(detection->bb3d, &pointCoord))
      {
        if(detection->box.type == "Car")
        {
          point->r = 255;
          point->g = 255;
          point->b = 0;
        }
        else if(detection->box.type == "Pedestrian")
        {
          point->r = 255;
          point->g = 0;
          point->b = 255;
        }
        else if(detection->box.type == "Van")
        {
          point->r = 0;
          point->g = 255;
          point->b = 255;
        }
        else if(detection->box.type == "Cyclist")
        {
          point->r = 0;
          point->g = 0;
          point->b = 255;
        }
        else
        {
          point->r = 0;
          point->g = 255;
          point->b = 0;
        }
      }
    }
  }
  for(auto detection = detections.begin(); detection < detections.end(); detection++)
  {
      delete detection->bb3d;
  }
}

void KittiParser::calcBBforDetection(tDetection* det, Eigen::MatrixXf* velo2cam, Eigen::MatrixXf* r0)
{
   double angle = det->ry;
  Eigen::Matrix3f RotY;
  RotY << std::cos(angle), 0, std::sin(angle),
                        0, 1,              0,
          -std::sin(angle), 0, std::cos(angle);

  Eigen::MatrixXf corners(3, 8);
  Eigen::MatrixXf corners3D(3, 8);
  Eigen::MatrixXf transsCorners(8, 3);
  Eigen::Vector3f coord(det->t1, det->t2, det->t3);
  Eigen::Matrix3f r0Inv = r0->inverse();
  det->bb3d = new Eigen::MatrixXf(8, 3);
  Eigen::MatrixXf veloTrans(4, 3);
  float l = det->l;
  float w = det->w;
  float h = det->h;
  corners << l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2,
               0,   0,    0,    0,  -h,  -h,   -h,   -h,
             w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2;
  corners3D = RotY * corners;

  for(int i = 0; i < 3; i++)
  {
      for (int j = 0; j < 8; j++)
      {
          corners3D(i, j) += coord(i);
      }
  }
  transsCorners = corners3D.transpose();
  Eigen::MatrixXf rect2ref = r0Inv * corners3D;
  rect2ref.transposeInPlace();
  Eigen::VectorXf ones = Eigen::VectorXf::Ones(8);
  Eigen::MatrixXf stackedRect2Ref(8, 4);
  stackedRect2Ref<<rect2ref, ones;
  veloTrans = velo2cam->transpose();
  *(det->bb3d) = stackedRect2Ref * veloTrans;
}

void KittiParser::saveToPcdFile(std::string file_path)
{
  PointCloudRGB out;
  for(auto point = cloud.points.begin(); point < cloud.points.end(); point++)
  {
    pcl::PointXYZRGB p;
    p.x = point->x;
    p.y = point->y;
    p.z = point->z;
    p.r = point->r;
    p.g = point->g;
    p.b = point->b;
    out.points.push_back(p);
  }
  out.width = cloud.points.size();
  out.height = 1;
  out.points.resize(out.width * out.height);
  pcl::io::savePCDFile(file_path, out);
  std::cout<<"Saving file ended"<<std::endl;
}

void inverseRigidTrans(Eigen::MatrixXf* matrix)
{
    Eigen::Matrix3f rot_matrix;
    Eigen::Matrix3f transp_rot_matrix;
    Eigen::Vector3f trans_vect;
    Eigen::Vector3f temp_vect;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            rot_matrix(i, j) = (*matrix)(i, j);
        }
    }
    for(int i = 0; i < 3; i++)
    {
        trans_vect(i) = -(*matrix)(i, 3);
    }
    transp_rot_matrix = rot_matrix.transpose();
    temp_vect = transp_rot_matrix * trans_vect ;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            (*matrix)(i, j) = transp_rot_matrix(i, j);
        }
    }
    for(int i = 0; i < 3; i++)
    {
        (*matrix)(i, 3) = temp_vect(i);
    }
}
bool isPointInBB(Eigen::MatrixXf* bb, Eigen::Vector3f* point)
{
   float xmax, ymax, zmax, xmin, ymin, zmin;
   xmax = ymax = zmax = -std::numeric_limits<float>::max();
   xmin = ymin = zmin = std::numeric_limits<float>::max();
   for(int i = 0; i < 8; i++)
   {
       xmax = xmax < (*bb)(i, 0)  ? (*bb)(i, 0) : xmax;
       xmin = xmin > (*bb)(i, 0)  ? (*bb)(i, 0) : xmin;

       ymax = ymax < (*bb)(i, 1)  ? (*bb)(i, 1) : ymax;
       ymin = ymin > (*bb)(i, 1)  ? (*bb)(i, 1) : ymin;

       zmax = zmax < (*bb)(i, 2)  ? (*bb)(i, 2) : zmax;
       zmin = zmin > (*bb)(i, 2)  ? (*bb)(i, 2) : zmin;
   }


    return (*point)(0) > xmin && (*point)(0) < xmax &&
           (*point)(1) > ymin && (*point)(1) < ymax &&
           (*point)(2) > zmin && (*point)(2) < zmax;
}
int main()
{
  std::string calibFile =  "../../input/tracking/training/calib/0000.txt";
  Eigen::MatrixXf* velo2cam = KittiParser::loadCalibrationFile(calibFile, "Tr_velo_cam", 4, 3);
  inverseRigidTrans(velo2cam);
  Eigen::MatrixXf* r0 = KittiParser::loadCalibrationFile(calibFile, "R_rect", 3, 3);
  for(int i = 0; i <= 153; i++)
  {
    char buffer[255];
    std::string point_input, label_input;
    std::snprintf(buffer, sizeof(buffer), "../../input/tracking/training/velodyne/0000/000%.3d.bin", i);
    point_input = buffer;
    std::snprintf(buffer, sizeof(buffer), "../../input/tracking/training/label_02/0000/000%.3d.txt", i);
    label_input = buffer;
    std::string out = "out/out" + std::to_string(i) + ".pcd";
    std::cout<<"Processing files started"<<std::endl;
    std::cout<<"Input file: "<<point_input<<std::endl<<"Label file: "<<label_input<<std::endl;
    KittiParser parser(point_input);
    parser.loadDetectionsFromTxt(label_input, velo2cam, r0);
    parser.saveToPcdFile(out);
    std::cout<<"Processing files finished"<<std::endl;
    std::cout<<"----------------------------------------------------------"<<std::endl;
  }
}
