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
//#include <pcl/ros/conversions.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>

#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageMapper3D.h>
#include <vtkImageMapToColors.h>
#include <vtkImageProperty.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkVersion.h>
#include <vtkImageViewer2.h>
#include <vtkPNGWriter.h>
#include <vtkImageShiftScale.h>
#include <vtkImageCast.h>

#include "pcloud.h"

using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace oscl;

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

  // Pcloud
  PCloud pcloud;
  
  // Load cloud in blob format
  //sensor_msgs::PointCloud2 blob;
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile (infile.c_str(), blob);

  pcl::PointCloud<PointXYZI>::Ptr cloud (new pcl::PointCloud<PointXYZI>);
  cout << "Loading point cloud...";
  //pcl::fromROSMsg (blob, *cloud);
  pcl::fromPCLPointCloud2(blob, *cloud);
  cout << "done." << endl;

  pcl::RangeImage range_image;

  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  0.2f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
  float noiseLevel = 0.00;
  float minRange = 0.0f;
  int borderSize = 1;

  range_image.createFromPointCloud(*cloud.get(), angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  //range_image.cropImage();

  // save range image
  int width = range_image.width;
  int height = range_image.height;
  cout << width << "  " << height << endl;
  // Create a "grayscale" 16x16 image, 1-component pixels of type "double"
  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  int imageExtent[6] = { 0, width-1, 0, height-1, 0, 0 };
  image->SetExtent(imageExtent);

#if VTK_MAJOR_VERSION <= 5
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToDouble();
#else
  image->AllocateScalars(VTK_DOUBLE, 1);
#endif 
  //double scalarvalue = 0.0;
  float *pixel_value = range_image.getRangesArray();
  float max = 0, min = 6000;
  for (int y = imageExtent[2]; y <= imageExtent[3]; y++)
    {
      for (int x = imageExtent[0]; x <= imageExtent[1]; x++)
	{
	  double* pixel = static_cast<double*>(image->GetScalarPointer(x, y, 0));
	  
	  if( pixel_value[x + y * width] < 0.0)
	    pixel[0] = 0;
	  else {
	    pixel[0] = pixel_value[x + y * width];
	    if(pixel[0] > max) max = pixel[0];
	    if(pixel[0] < min) min = pixel[0];
	  }
	}
    }

  // Map the scalar values in the image to colors with a lookup table:
  vtkSmartPointer<vtkLookupTable> lookupTable =
    vtkSmartPointer<vtkLookupTable>::New();
  lookupTable->SetNumberOfTableValues(500);
  lookupTable->SetRange(min, max);
  lookupTable->Build();
 
  // Pass the original image and the lookup table to a filter to create
  // a color image:
  vtkSmartPointer<vtkImageMapToColors> scalarValuesToColors =
    vtkSmartPointer<vtkImageMapToColors>::New();
  scalarValuesToColors->SetLookupTable(lookupTable);
  scalarValuesToColors->PassAlphaToOutputOn();
#if VTK_MAJOR_VERSION <= 5
  scalarValuesToColors->SetInput(image);
#else
  scalarValuesToColors->SetInputData(image);
#endif
  scalarValuesToColors->SetOutputFormatToRGB();

  // Output
  // set file name
  std::string fname("RangeImage.");
  std::stringstream ss;
  ss << 1;
  fname = fname + ss.str() + ".png";
  vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(fname.c_str());
  writer->SetInputConnection(scalarValuesToColors->GetOutputPort());
  writer->Write();
  std::cout << "Saved RangeImage." << 1 << ".png" << std::endl;


  // --------------------------
  // -----Show range image-----
  // --------------------------
  // pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  // pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  // viewer.setBackgroundColor (1, 1, 1);
  // // boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
  // viewer.initCameraParameters ();

  // *range_image_ptr = range_image;
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);



  // range_image_widget.showRangeImage (range_image, -std::numeric_limits<float>::infinity (),
  // 				     std::numeric_limits<float>::infinity (),
  // 				     true );
  // range_image_widget.setSize(700,700);

  // cout << range_image << endl;
  // while (!viewer.wasStopped ())
  //   {
  
  // //     viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  // //     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  // //     pcloud.setViewerPose(viewer, range_image.getTransformationToWorldSystem ());

            
  // //     viewer.spinOnce();
  //     range_image_widget.spinOnce ();
  //   }
  
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  // viewer->addCoordinateSystem (1.0);
  // viewer->initCameraParameters ();

  //--------------------
  // -----Main loop-----
  //--------------------
  // while (!viewer->wasStopped ())
  //   {
  //     viewer->spinOnce (100);
  //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //   }

  
  return 0;
}




// #include <iostream>

// #include <boost/thread/thread.hpp>

// #include <pcl/common/common_headers.h>
// #include <pcl/range_image/range_image.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/range_image_visualizer.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

// typedef pcl::PointXYZ PointType;

// // --------------------
// // -----Parameters-----
// // --------------------
// float angular_resolution_x = 0.5f,
//       angular_resolution_y = angular_resolution_x;
// pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
// bool live_update = false;

// // --------------
// // -----Help-----
// // --------------
// void 
// printUsage (const char* progName)
// {
//   std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
//             << "Options:\n"
//             << "-------------------------------------------\n"
//             << "-rx <float>  angular resolution in degrees (default "<<angular_resolution_x<<")\n"
//             << "-ry <float>  angular resolution in degrees (default "<<angular_resolution_y<<")\n"
//             << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
//             << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
//             << "-h           this help\n"
//             << "\n\n";
// }

// void 
// setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
// {
//   Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
//   Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
//   Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
//   viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
//                             look_at_vector[0], look_at_vector[1], look_at_vector[2],
//                             up_vector[0], up_vector[1], up_vector[2]);
// }

// // --------------
// // -----Main-----
// // --------------
// int 
// main (int argc, char** argv)
// {
//   // --------------------------------------
//   // -----Parse Command Line Arguments-----
//   // --------------------------------------
//   if (pcl::console::find_argument (argc, argv, "-h") >= 0)
//   {
//     printUsage (argv[0]);
//     return 0;
//   }
//   if (pcl::console::find_argument (argc, argv, "-l") >= 0)
//   {
//     live_update = true;
//     std::cout << "Live update is on.\n";
//   }
//   if (pcl::console::parse (argc, argv, "-rx", angular_resolution_x) >= 0)
//     std::cout << "Setting angular resolution in x-direction to "<<angular_resolution_x<<"deg.\n";
//   if (pcl::console::parse (argc, argv, "-ry", angular_resolution_y) >= 0)
//     std::cout << "Setting angular resolution in y-direction to "<<angular_resolution_y<<"deg.\n";
//   int tmp_coordinate_frame;
//   if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
//   {
//     coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
//     std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
//   }
//   angular_resolution_x = pcl::deg2rad (angular_resolution_x);
//   angular_resolution_y = pcl::deg2rad (angular_resolution_y);
  
//   // ------------------------------------------------------------------
//   // -----Read pcd file or create example point cloud if not given-----
//   // ------------------------------------------------------------------
//   pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
//   pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
//   Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//   std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
//   if (!pcd_filename_indices.empty ())
//   {
//     std::string filename = argv[pcd_filename_indices[0]];
//     if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
//     {
//       std::cout << "Was not able to open file \""<<filename<<"\".\n";
//       printUsage (argv[0]);
//       return 0;
//     }
//     scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
//                                                              point_cloud.sensor_origin_[1],
//                                                              point_cloud.sensor_origin_[2])) *
//                         Eigen::Affine3f (point_cloud.sensor_orientation_);
//   }
//   else
//   {
//     std::cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
//     for (float x=-0.5f; x<=0.5f; x+=0.01f)
//     {
//       for (float y=-0.5f; y<=0.5f; y+=0.01f)
//       {
//         PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
//         point_cloud.points.push_back (point);
//       }
//     }
//     point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
//   }
  
//   // -----------------------------------------------
//   // -----Create RangeImage from the PointCloud-----
//   // -----------------------------------------------
//   float noise_level = 0.0;
//   float min_range = 0.0f;
//   int border_size = 1;
//   boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
//   pcl::RangeImage& range_image = *range_image_ptr;   
//   range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
//                                     pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//   viewer.setBackgroundColor (1, 1, 1);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
//   viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
//   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
//   //viewer.addCoordinateSystem (1.0f, "global");
//   //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
//   //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
//   viewer.initCameraParameters ();
//   setViewerPose(viewer, range_image.getTransformationToWorldSystem ());
  
//   // --------------------------
//   // -----Show range image-----
//   // --------------------------
//   pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//   range_image_widget.showRangeImage (range_image);
  
//   //--------------------
//   // -----Main loop-----
//   //--------------------
//   while (!viewer.wasStopped ())
//   {
//     range_image_widget.spinOnce ();
//     viewer.spinOnce ();
//     pcl_sleep (0.01);
    
//     if (live_update)
//     {
//       scene_sensor_pose = viewer.getViewerPose();
//       range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
//                                         pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                         scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
//       range_image_widget.showRangeImage (range_image);
//     }
//   }
// }
// 
