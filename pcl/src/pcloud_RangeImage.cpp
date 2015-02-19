#include "pcloud.h"

#include <pcl/visualization/range_image_visualizer.h>

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


void onlineclust::PCloud::getRangeImage(_pclType1::Ptr &pcloud, std::vector<pcl::PointIndices> &cluster_indices, pcl::RangeImage *&range_image, uint &rangeImage_num)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    std::cout << "no. of range images: " << cluster_indices.size() << std::endl;
    // --------------------------------------------
    // -----save sub-clusters in rangeImage[]-----
    // --------------------------------------------
    range_image = new pcl::RangeImage[cluster_indices.size()];
    rangeImage_num = cluster_indices.size();

    uint i = 0;
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // extract sub-clusters
        _pclType1::Ptr sub_cluster(new _pclType1);

        extract.setInputCloud (pcloud);
        extract.setIndices( boost::shared_ptr<pcl::PointIndices>( new pcl::PointIndices( *it ) ) );
        extract.setNegative (false);
        extract.filter (*sub_cluster);

        // We now want to create a range image from the above point cloud, with a 1deg angular resolution
        float angularResolution = (float) (  0.1f * (M_PI/180.0f));  //   1.0 degree in radians
        float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
        float noiseLevel = 0.00;
        float minRange = 0.0f;
        int borderSize = 1;

        range_image[i].createFromPointCloud(*sub_cluster.get(), angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

        //range_image[i].createFromPointCloudWithViewpoints(*sub_cluster.get(), angularResolution, maxAngleWidth, maxAngleHeight,
        //                                                  coordinate_frame, noiseLevel, minRange, borderSize);

        i++;
    }
}

void onlineclust::PCloud::vis_pointcloud2rangeimage(_pclType1::Ptr &pcloud, std::vector<pcl::PointIndices> &cluster_indices)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    std::cout << "no. of clusters: " << cluster_indices.size() << std::endl;
    // --------------------------------------------
    // -----save sub-clusters in rangeImage[]-----
    // --------------------------------------------
    pcl::RangeImage *range_image = new pcl::RangeImage[cluster_indices.size()];

    uint i = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // extract sub-clusters
        _pclType1::Ptr sub_cluster(new _pclType1);

        extract.setInputCloud (pcloud);
        extract.setIndices( boost::shared_ptr<pcl::PointIndices>( new pcl::PointIndices( *it ) ) );
        extract.setNegative (false);
        extract.filter (*sub_cluster);

        // We now want to create a range image from the above point cloud, with a 1deg angular resolution
        float angularResolution = (float) (  0.25f * (M_PI/180.0f));  //   1.0 degree in radians
        float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
        float noiseLevel = 0.00;
        float minRange = 0.0f;
        int borderSize = 1;

        range_image[i].createFromPointCloud(*sub_cluster.get(), angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

        //range_image[i].createFromPointCloudWithViewpoints(*sub_cluster.get(), angularResolution, maxAngleWidth, maxAngleHeight,
        //                                                  coordinate_frame, noiseLevel, minRange, borderSize);
	delete[] range_image;
        i++;
    }

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);

    //viewer.addCoordinateSystem (1.0f, "global");
    //pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters ();

    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");

    //--------------------
    // -----Main loop-----
    //--------------------
    i = 0;
    while (!viewer.wasStopped ())
    {

      if (i < cluster_indices.size())
	{
          range_image_widget.spinOnce ();
          viewer.spinOnce ();
          pcl_sleep (1);

	  //scene_sensor_pose = viewer.getViewerPose();
	  *range_image_ptr = range_image[i];

	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
	  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

	  setViewerPose(viewer, range_image[i].getTransformationToWorldSystem ());
	  range_image_widget.setSize(500,500);
	  range_image_widget.showRangeImage (range_image[i], -std::numeric_limits<float>::infinity (),
					     std::numeric_limits<float>::infinity (),
					     true );
	  i++;

	}
      else
	{
          range_image_widget.spinOnce (100);
          viewer.spinOnce (100);
	}

    }
  
    std::cout << "Start to save .png files" << std::endl;
    for(i = 0; i < cluster_indices.size(); ++i)
      {
	// store range_image[0]
	int width = range_image[i].width;
	int height = range_image[i].height;

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
	float *pixel_value = range_image[i].getRangesArray();
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
	ss << i;
	fname = fname + ss.str() + ".png";
	vtkSmartPointer<vtkPNGWriter> writer =
	  vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(fname.c_str());
	writer->SetInputConnection(scalarValuesToColors->GetOutputPort());
	writer->Write();
	std::cout << "Saved RangeImage." << i << ".png" << std::endl;
      }
}


