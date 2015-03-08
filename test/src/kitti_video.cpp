#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include "pcloud.h"
#include "proc.h"

using namespace std;
using namespace oscl::engine;
using namespace oscl;

pcl::visualization::Camera camera;

void init_camera(pcl::visualization::Camera&);
//void 

int main(int argc, char**argv){

  if(argc != 2){
    cout << "Usage: <./exec> <input_dir>\n";
      return -1;
  }

  PCloud pcloud;
  
  vector<string> allpcd;

  /* get all .pcd files directory */
  sub_dir_files(argv[1], ".pcd", allpcd);
  
  /* set viewer */
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.setBackgroundColor (0, 0, 0);

  /* set camera position */
  pcl::visualization::Camera camera;
  init_camera(camera);
  
  viewer.setCameraParameters(camera);
  //viewer.updateCamera();
  
  uint i = 0;
  while(!viewer.wasStopped()){

    if(i < allpcd.size()){

      /* load pcloud from file*/
      allpcd[i] = "/" + allpcd[i];
      cout << allpcd[i] << endl;
      pcl::PointCloud<pcl::PointXYZI>::Ptr icloud (new pcl::PointCloud<pcl::PointXYZI>),
	ocloud(new pcl::PointCloud<pcl::PointXYZI>);

      pcl::io::loadPCDFile<pcl::PointXYZI> (allpcd[i].c_str(), *icloud);

      // seg plane model
      pcloud.plane_model_segmentation(*icloud, ocloud);

      // remove all points before adding
      viewer.removeAllPointClouds();
            
      // color handle
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tot_pcloud_color (ocloud, 255, 255, 255);

      // 
      viewer.addPointCloud<pcl::PointXYZI>(ocloud, tot_pcloud_color, allpcd[i].c_str());
      //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, allpcd[i].c_str());
      
      viewer.spinOnce();
      
      ++i;
    } else {
      viewer.spinOnce();
    }

    
  }
  
  
  return 0;


}

void init_camera(pcl::visualization::Camera& camera){

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
