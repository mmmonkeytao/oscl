#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ros/conversions.h>

#include <iostream>
#include <fstream>

using namespace pcl;
using namespace std;

namespace po = boost::program_options;


int main(int argc, char** argv)
{
  string infile;

  po::options_description desc("Program options");
  desc.add_options()
    ("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from");

  // Parse the command line
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // Print help
  if (vm.count("help"))
    {
      cout << desc << "\n";
      return false;
    }

  // Process options.
  po::notify(vm);

  // Load cloud in blob format
  //sensor_msgs::PointCloud2 blob;
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile (infile.c_str(), blob);

  pcl::PointCloud<PointXYZI>::Ptr cloud (new pcl::PointCloud<PointXYZI>);
  cout << "Loading point cloud...";
  //pcl::fromROSMsg (blob, *cloud);
  pcl::fromPCLPointCloud2(blob, *cloud);
  cout << "done." << endl;

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

  
  return 0;
}
