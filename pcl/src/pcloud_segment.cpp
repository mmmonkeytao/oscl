#include "pcloud.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/don.h>
#include <pcl/segmentation/sac_segmentation.h>


void oscl::PCloud::euclidean_cluster_extraction(_pclType1::Ptr &pcloud,  std::vector<pcl::PointIndices>&cluster_indices)
{
      // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (pcloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (this->fparam.segradius);
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pcloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      _pclType1::Ptr cloud_cluster (new _pclType1);
      for (auto pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            //cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
            pcloud->points[*pit].intensity = j/(double)cluster_indices.size() ;
        }
      /*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::      std::cout << "PointCloud representing the Cluster: No." << j << " " << cloud_cluster->points.size () << " data points." << std::std::endl;
      */
      j++;
    }

}

void oscl::PCloud::diff_normal_segmentation(_pclType1& icloud, _pclType1::Ptr& pcloud, uint ith)
{
 // filters, remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (icloud.makeShared());
    sor.setMeanK (this->fparam.NUM_NEIGHBORS);
    sor.setStddevMulThresh (this->fparam.STD_DEVIATION);
    //sor.filter (*cloud_filtered);
    sor.filter (*(icloud.makeShared()));

    std::cout << "Start processing Point Clouds: " << ith << std::endl;

    /* Difference of Normals Based Segmentation */
    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZI>::Ptr tree;
    if (icloud.isOrganized())
    {
      std::cout << "Points Cloud " << ith << " is organized." << std::endl;
      tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZI> ());
    }
    else
    {
      std::cout << "Points Cloud " << ith << " is unorganized." << std::endl;
      tree.reset (new pcl::search::KdTree<pcl::PointXYZI> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (icloud.makeShared());

    if(this->fparam.scale1 >= this->fparam.scale2)
    {
      std::cerr << "Error: Large scale must be > small scale!" << std::endl;
      exit (EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointNormal> ne;
    ne.setInputCloud (icloud.makeShared());
    ne.setSearchMethod (tree);

    /**
    * NOTE: setting viewpoint is very important, so that we can ensure
    * normals are all pointed in the same direction!
    */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    std::cout << "Calculating normals for scale..." << this->fparam.scale1 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (this->fparam.scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    std::cout << "Calculating normals for scale..." << this->fparam.scale2 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (this->fparam.scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZI, pcl::PointNormal>(*(icloud.makeShared()), *doncloud);

          std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (icloud.makeShared());
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Filter by magnitude
    std::cout << "Filtering out DoN mag <= " << this->fparam.threshold << "  " << std::endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
                new pcl::ConditionOr<pcl::PointNormal> ()
                );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, this->fparam.threshold)));

    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Filter by magnitude
    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << this->fparam.segradius << "..." << std::endl;

    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

    ec.setClusterTolerance (this->fparam.segradius);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (4000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr ocloud (new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    {

        for (auto pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            //cloud_cluster_don->points.push_back (doncloud->points[*pit]);
            pcl::PointXYZI point;
            point.x = doncloud->points[*pit].x;
            point.y = doncloud->points[*pit].y;
            point.z = doncloud->points[*pit].z;
            point.intensity = j;

            pcloud->points.push_back(point);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;

    }

    //pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZI>(*doncloud, *cloud_filtered);
    // filter again
    /*sor.setInputCloud (ccloud);
    sor.setMeanK (NUM_NEIGHBORS);
    sor.setStddevMulThresh (STD_DEVIATION);
    sor.filter (*ccloud);*/
  
}

void oscl::PCloud::plane_model_segmentation(_pclType1 &cloud, _pclType1::Ptr &pcloud)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(this->fparam.distance_threshold);

    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "(plane_model_seg)Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    /*for (size_t i = 0; i < inliers->indices.size (); ++i)
      std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                 << cloud->points[inliers->indices[i]].y << " "
                                                 << cloud->points[inliers->indices[i]].z << std::std::endl;*/


    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    // Extract the inliers
    extract.setInputCloud (cloud.makeShared());
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*pcloud);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::std::endl;

    //std::stringstream ss;
    //ss << "table_scene_lms400_plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZI> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*pcloud);
    //cloud_filtered.swap (cloud_f);

}


