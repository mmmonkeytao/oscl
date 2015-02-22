#ifndef __PCLOUD_H__
#define __PCLOUD_H__

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>

namespace oscl{
  
  using _pclType1 = pcl::PointCloud<pcl::PointXYZI>;
  
  class PCloud{
    
  public:

    PCloud(): cloud(nullptr), camera(pcl::visualization::Camera()){}; 
    /// 
    /// NOTE: this function will allocate memories of 
    /// attribute *cloud
    /// @param fpcl 
    /// @param fnum 
    ///
    void load_pcloud(const char *fpcl, int fnum);
    /// 
    ///
    /// @param cloud 
    /// @param ith 
    ///
    /// @return 
    ///
    void proc_pcloud(_pclType1::Ptr& pcloud,uint ith = 0);
    /// 
    ///
    /// @param cloud 
    /// @param scloud 
    /// @param ith 
    ///
    void diff_normal_segmentation(_pclType1 &cloud, _pclType1::Ptr &scloud, uint ith = 0);
    /// 
    ///
    /// @param cloud 
    /// @param ocloud 
    ///
    void plane_model_segmentation(_pclType1 &cloud, _pclType1::Ptr &ocloud);
    /// 
    ///
    /// @param pcloud 
    /// @param cluster_indices 
    ///
    void euclidean_cluster_extraction(_pclType1::Ptr &pcloud,std::vector<pcl::PointIndices>&cluster_indices);
    /// 
    ///
    /// @param pcloud 
    /// @param cluster_indices 
    ///
    void vis_pointcloud2rangeimage(_pclType1::Ptr &pcloud,std::vector<pcl::PointIndices> &cluster_indices);


    void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
    /// 
    ///
    /// @param cloud 
    /// @param cloud_normals 
    ///
    void cloud_normal(_pclType1::Ptr &pcloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);
    /// 
    ///
    /// @param pcloud 
    /// @param cluster_indices 
    /// @param range_image 
    /// @param rangeImage_num 
    ///
    void getRangeImage(_pclType1::Ptr &pcloud, std::vector<pcl::PointIndices> &cluster_indices, pcl::RangeImage *&range_image, uint &rangeImage_num);

    _pclType1& getCloud(uint ith) const{ return cloud[ith]; };
    /// 
    ///
    ///
    /// @return 
    ///
    const pcl::visualization::Camera &getCamera()const{return camera;}
    
    void initCamParam(){
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
    
    struct Fparam{
      // diff norm segmentation
      double scale1 = 0.1, 
	scale2 = 0.8, 
	radius = 0.9, 
	STD_DEVIATION = 0.6, 
	threshold = 0.13;
      uint   NUM_NEIGHBORS = 45; 
      // plane model segmentation
      double distance_threshold = 0.5;
      // clustering for pcloud
      double segradius = 0.5;
    }fparam;

    
    ~PCloud()
    {
      if(cloud)
	delete[] cloud;
    }
    
  private:
    _pclType1 *cloud;
    // camera position parameters
     pcl::visualization::Camera camera;

  };

}

#endif
