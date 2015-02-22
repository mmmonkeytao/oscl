#include "pcloud.h"

#include <stdexcept>
#include <cstdio>
#include <string>
#include <sstream>

#include <pcl/search/kdtree.h>	
#include <pcl/segmentation/conditional_euclidean_clustering.h>


void oscl::PCloud::load_pcloud(const char *dir, int nfiles)
{
  this->cloud = new _pclType1[nfiles];

  if(nfiles == 1){
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (dir, cloud[0]) == -1)
    {
      std::cout << "\nCan't load file: ";
      throw std::runtime_error(dir);
    }
    else
    {
      std::cout << "Successfully load pcl data file: " << dir << std::endl;
    }
    return;
  }
  
  for(uint i = 0; i < nfiles; ++i ){
    std::string infile(dir);
    std::stringstream ss;
    ss << i;
    infile = infile + "kitti." + ss.str() + ".pcd";
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (infile.c_str(), cloud[i]) == -1)
    {
      std::cout << "\nCan't load file: ";
      throw std::runtime_error(infile.c_str());
    }
    else
    {
      std::cout << "Successfully load pcl data file: " << infile << std::endl;
    }
  }

}

void oscl::PCloud::proc_pcloud(_pclType1::Ptr& pcloud, uint ith)
{
  // segmentation using different normals
  // this->diff_normal_segmentation(ocloud, pcloud, ith);
  //pcloud = new _pclType1{this->cloud[ith]};

  // seg plane model
  plane_model_segmentation(this->cloud[ith], pcloud);
  
  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  euclidean_cluster_extraction(pcloud, cluster_indices);
  //vis_pointcloud2rangeimage(pcloud, cluster_indices);
	    
  // calculate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  cloud_normal(pcloud, cloud_normals);

}

void oscl::PCloud::cloud_normal(_pclType1::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
{

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne1;
    ne1.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    ne1.setSearchMethod (tree);

    ne1.setRadiusSearch (this->fparam.radius);
    ne1.compute (*cloud_normals);
}


void oscl::PCloud::setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}


