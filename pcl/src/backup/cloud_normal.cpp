#include "pcl_func.hpp"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

void cloud_normal(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, double radius)
{

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne1;
    ne1.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    ne1.setSearchMethod (tree);

    ne1.setRadiusSearch (radius);
    ne1.compute (*cloud_normals);
}
