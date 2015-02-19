#ifndef PCL_FUNC_HPP
#define PCL_FUNC_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void vis_pointcloud2rangeimage(pcl::PointCloud<pcl::PointXYZI>::Ptr &, std::vector<pcl::PointIndices> &);
void setViewerPose (pcl::visualization::PCLVisualizer& , const Eigen::Affine3f& );
void read_parameters(const char *file_name, double &, double &, double &, double &, int&, double&, double&, double &);
void diff_normal_segmentation(pcl::PointCloud<pcl::PointXYZI> &, pcl::PointCloud<pcl::PointXYZI>::Ptr &, double, double, double, double, double, double, double, int);
void plane_model_segmentation( pcl::PointCloud<pcl::PointXYZI> &, pcl::PointCloud<pcl::PointXYZI>::Ptr &, double);
void cloud_normal(pcl::PointCloud<pcl::PointXYZI>::Ptr &, pcl::PointCloud<pcl::Normal>::Ptr &, double);
void euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr &, std::vector<pcl::PointIndices> &, double );

#endif // PCL_FUNC_HPP
