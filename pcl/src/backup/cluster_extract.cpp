#include "pcl_func.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

void euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, std::vector<pcl::PointIndices> &cluster_indices, double segradius)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_in);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            //cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
            cloud_in->points[*pit].intensity = j/(double)cluster_indices.size() ;
        }
      /*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: No." << j << " " << cloud_cluster->points.size () << " data points." << std::endl;
      */
      j++;
    }

}
