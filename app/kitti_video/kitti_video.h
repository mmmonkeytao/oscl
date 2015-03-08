#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <boost/program_options.hpp>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "pcloud.h"
#include "proc.h"
#include "tracklets.h"
#include "hmp.h"
#include "Galaxy.h"

long int fileID = 10000000000;

using namespace std;
using namespace oscl::engine;
using namespace oscl;
using namespace pcl;
using namespace cv;

std::string objtype = "all";

enum OBJ_TYPE {
  Car = 0,
  Van = 1,
  Truck = 2,
  Pedestrian = 3,
  Sitter = 4,
  Cyclist = 5,
  Tram = 6,
  Misc = 7
};

struct COLOR{

  COLOR(){};
  COLOR(uint8_t r, uint8_t g, uint8_t b): R(r), G(g), B(b){}
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0; 
};

COLOR CCar = {255, 0, 0};  // Red
COLOR CVan = {0, 255, 0};  // Lime
COLOR CTruck = {0,0,255}; //Blue
COLOR CPedestrian = {255,255,0}; //Yellow
COLOR CSitter = {255,0,255};  //Magenta
COLOR CCyclist = {0,255,255}; // cyan
COLOR CTram = {238,130,238}; // violet
COLOR CMisc = {160,82,45};  // sienna

void init_type2color(map<OBJ_TYPE, COLOR> &type2color)
{
  type2color[OBJ_TYPE::Car] = CCar;
  type2color[OBJ_TYPE::Van] = CVan;
  type2color[OBJ_TYPE::Truck] = CTruck;
  type2color[OBJ_TYPE::Pedestrian] = CPedestrian;
  type2color[OBJ_TYPE::Sitter] = CSitter;
  type2color[OBJ_TYPE::Cyclist] = CCyclist;
  type2color[OBJ_TYPE::Tram] = CTram;
  type2color[OBJ_TYPE::Misc] = CMisc;
}

OBJ_TYPE str2type(string& type){

  if(!type.compare("Car")){
    return OBJ_TYPE::Car;
  }
  else if(!type.compare("Van")){
    return OBJ_TYPE::Van;
  }
  else if(!type.compare("Truck")){
    return OBJ_TYPE::Truck;
  }
  else if(!type.compare("Pedestrian")){
    return OBJ_TYPE::Pedestrian;
  }
  else if(!type.compare("Person (sitting)")){
    return OBJ_TYPE::Sitter;
  }
  else if(!type.compare("Cyclist")){
    return OBJ_TYPE::Cyclist;
  }
  else if(!type.compare("Tram")){
    return OBJ_TYPE::Tram;
  }
  else if(!type.compare("Misc")){
    return OBJ_TYPE::Misc;
  }
  
}

void init_str2color(map<string, COLOR>& str2color){

  str2color["Car"] = CCar;
  str2color["Van"] = CVan;
  str2color["Truck"] = CTruck;
  str2color["Pedestrian"] = CPedestrian;
  str2color["Person (sitting)"] = CSitter;
  str2color["Cyclist"] = CCyclist;
  str2color["Tram"] = CTram;
  str2color["Misc"] = CMisc;
}

void init_camera(pcl::visualization::Camera& camera){

  // camera.clip[0] = 28.1809;
  // camera.clip[1] = 159.088;
  // camera.focal[0] = -1;
  // camera.focal[1] = 0;
  // camera.focal[2] = 0;
  // camera.pos[0] = -26.1027; 
  // camera.pos[1] = 2.35786;
  // camera.pos[2] = -0.319249;
  // camera.view[0] = -0.0116619; 
  // camera.view[1] = 0.0112222;
  // camera.view[2] =  0.999869;
  // camera.fovy = 0.523599;
  // camera.window_size[0] = 1200;
  // camera.window_size[1] =650;
  // camera.window_pos[0] = 61;
  // camera.window_pos[1] = 52;

  camera.clip[0] = 28.1809;
  camera.clip[1] = 159.088;
  camera.focal[0] = 0;
  camera.focal[1] = 0;
  camera.focal[2] = 0;
  camera.pos[0] = -9.43671;
  camera.pos[1] = -70.3455;
  camera.pos[2] = 37.2992;
  camera.view[0] = 0.0316502;
  camera.view[1] = 0.464899;
  camera.view[2] = 0.884798;
  camera.fovy = 0.523599;
  camera.window_size[0] = 1200;
  camera.window_size[1] =650;
  camera.window_pos[0] = 61;
  camera.window_pos[1] = 52;
}


void trackletextract(pcl::visualization::PCLVisualizer &viewer,
		     pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
		     Tracklets *& tracklets,
		     uint frameid);

void addboundingbox(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string id);

void pcd2RangeImage(pcl::PointCloud<PointXYZI>::Ptr &cloud, pcl::RangeImage &range_image);

void RangeImage2cvMatrix(pcl::RangeImage &range_image, cv::Mat&, cv::Mat&);
