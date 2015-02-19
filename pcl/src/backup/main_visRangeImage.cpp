#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstring>
#include <stdlib.h>
#include "pcl_func.hpp"

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cout << "Usage: <./exec> <input_dir> <i-th_files>" << endl;
        return -1;
    }

    // read parameters
    int NUM_NEIGHBORS;
    double scale1, scale2, threshold, segradius, STD_DEVIATION, radius, distance_threshold;
    read_parameters("parameter.dat", scale1, scale2, threshold, segradius, NUM_NEIGHBORS, STD_DEVIATION, radius, distance_threshold);

    // Initialization
    int i;
    const int ith_file = atoi(argv[2]);
    pcl::PointCloud<pcl::PointXYZI>* cloud = new pcl::PointCloud<pcl::PointXYZI>();

    // load series files
    cout << endl;

    string infile(argv[1]);
    stringstream ss;
    ss << ith_file;
    infile = infile + "kitti." + ss.str() + ".pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (infile.c_str(), *cloud) == -1) //* load the file
      {
	cerr << "Couldn't read file: " << infile << endl;
	//PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	return (-1);
      }
    else
      {
	cout << "Successfully load file: " << infile << endl;
      }
    
    cout << endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ocloud (new pcl::PointCloud<pcl::PointXYZI>);

    // seg different normals
    //diff_normal_segmentation(cloud[i], ocloud, scale1, scale2, threshold, segradius, NUM_NEIGHBORS, STD_DEVIATION, radius, i);

    // seg plane model
    plane_model_segmentation( *cloud, ocloud, distance_threshold);

    // clustering
    std::vector<pcl::PointIndices> cluster_indices;
    euclidean_cluster_extraction(ocloud, cluster_indices, segradius);
    vis_pointcloud2rangeimage(ocloud, cluster_indices);

    // calculate normals
    //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //cloud_normal(ocloud, cloud_normals, radius);

    // remove all points before adding
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(ocloud, "intensity");
    //viewer.removeAllPointClouds();

    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    //viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (ocloud, cloud_normals, 80, 0.5, "normals");
    //viewer.addPointCloud<pcl::PointXYZI>(ocloud, intensity_distribution, "KITTI Viewer");
    //viewer.spinOnce();
      
    return 0;
}





