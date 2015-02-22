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
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

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

#include <vector>

#include <Eigen/Dense>
#include <chrono>

#include <iostream>
#include <fstream>
#include <sstream>

#include "pcloud.h"
#include "proc.h"

// Labels: 0 Cars, 1 Vans, 2 Trucks, 3 Pedestrians, 4 Sitters, 5 Cyclists, 6 Trams, 7 Misc

using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace oscl;
using namespace oscl::engine;
using namespace std::chrono;

namespace po = boost::program_options;

void pcd2RangeImage(pcl::PointCloud<PointXYZI>::Ptr &, pcl::RangeImage &);
void saveRangeImageInfo(pcl::RangeImage&, int label, const char* outfile);
void saveRangeImage(pcl::RangeImage&, const char* outfile);
int tag2label(string&);

int main(int argc, char** argv)
{

  string infile, outfile, tag;

  po::options_description desc("Program options");
  desc.add_options()
    ("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
    ("outfile", po::value<string>(&outfile)->required(), "the file to save a depth and normals to")
    ("tag", po::value<string>(&tag)->required(), "the tag of file");
  
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

  cout << "loading .pcd: " << infile << endl;
  pcl::fromPCLPointCloud2(blob, *cloud);

  int label = tag2label(tag);
  
  pcl::RangeImage range_image;
  pcd2RangeImage(cloud, range_image);

  if(range_image.width >= 15 && range_image.height >=15)
    {
      saveRangeImageInfo(range_image, label, outfile.c_str());
      saveRangeImage(range_image, outfile.c_str());      
    }
  else
    cout << "Range Image is too small, we don't save file: " << infile << endl;

  cout << "done." << endl;
  
  return 0;
}

void pcd2RangeImage(pcl::PointCloud<PointXYZI>::Ptr &cloud, pcl::RangeImage &range_image)
{

  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  0.2f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (360.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
  float noiseLevel = 0.00;
  float minRange = 0.0f;
  int borderSize = 1;

  range_image.createFromPointCloud(*cloud.get(), angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  range_image.cropImage();
}

void saveRangeImageInfo(pcl::RangeImage&range_image, int label, const char* outfile)
{

  int width = range_image.width;
  int height = range_image.height;
  cout << "width: " << width << "  " << "height: " << height << endl;

  float *pixel_value = range_image.getRangesArray();

  string opath(outfile);
  opath += ".dat";
  
  ofstream ofile(opath.c_str());
  
  ofile << height << " " << width << " " << label << endl;

  for(uint x = 0; x < width; ++x){
    for(uint y = 0; y < height; ++y){
      
      float var;

      if(pixel_value[x + y*height] < 0.0 || pixel_value[x + y*height] > 10.0e10 || isnan(pixel_value[x + y*height]))
	var = 0.0;
      else{
	var = pixel_value[x + y*height];
      }

      // depth normal
      Eigen::Vector3f normal;
      
      range_image.getNormal(x,y,5,normal);

      ofile << var << " " << normal.transpose() << endl;       
    }

  }
  ofile.close();
}

// Labels: 0 Cars, 1 Vans, 2 Trucks, 3 Pedestrians, 4 Sitters, 5 Cyclists, 6 Trams, 7 Misc
int tag2label(string &tag)
{
  if(!tag.compare("Car"))
    return 0;
  else if(!tag.compare("Van"))
    return 1;
  else if(!tag.compare("Truck"))
    return 2;
  else if(!tag.compare("Pedestrian"))
    return 3;
  else if(!tag.compare("Sitter"))
    return 4;
  else if(!tag.compare("Cyclist"))
    return 5;
  else if(!tag.compare("Tram"))
    return 6;
  else if(!tag.compare("Misc"))
    return 7;
}

void saveRangeImage(pcl::RangeImage& range_image, const char* outfile)
{
  // save range image
  int width = range_image.width;
  int height = range_image.height;
  cout <<"Range image size: " << width << "  " << height << endl;
  
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
  string ofile(outfile);
  string path = ofile + ".png";
  
  vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(path.c_str());
  writer->SetInputConnection(scalarValuesToColors->GetOutputPort());
  writer->Write();
}
