#include "kitti_video.h"

map<string, COLOR> ObjId2Color;
map<string, COLOR> str2color;
map<OBJ_TYPE, COLOR> type2color;

HMP hmp("hmp.config", "depth", "second+first");
Galaxy galaxy(71400, "CityBlock");

string video_text = "Groud Truth Label with Color:\nCar: Red\nVan: Lime\nTruck: Blue\nPedestrian: Yellow\nPerson(sitting): Magenta\nCyclist: Cyan\nTram: Violet\nMisc(background): Sienna";

string video_text2 = "KITTI 0091\n";

uint global_iter = 0;

int main(int argc, char**argv){

  if(argc != 2){
    cout << "Usage: <./exec> <input_dir>\n";
      return -1;
  }

  /* Initialization */
  // declare various object
  PCloud pcloud;
  init_str2color(str2color);
  init_type2color(type2color);
  
  /* get all .pcd files directory */
  vector<string> allpcd;
  sub_dir_files(argv[1], ".pcd", allpcd);
  uint NUM_FILES = allpcd.size();
  string file_prefix(argv[1]);
  
  /* set viewer */
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.setBackgroundColor (255, 255, 255);

  /* set camera position */
  pcl::visualization::Camera camera;
  init_camera(camera);
  
  viewer.setCameraParameters(camera);
  viewer.addCoordinateSystem(5.0);
  //viewer.updateCamera();

  // tracklet path
  string tracklet_path = file_prefix  + "tracklet_labels.xml";
  Tracklets *tracklets = new Tracklets();
  tracklets->loadFromFile(tracklet_path);

  uint i = 0;
  ObjId2Color = map<string, COLOR>();
  
  while(!viewer.wasStopped()){

    if(i < NUM_FILES){

      /* create pcd file name */
      stringstream ss;
      ss << (fileID+i) << ".pcd";
      string file_path = file_prefix + ss.str().substr(1, string::npos);
      cout << file_path << endl;

      /* load point clouds from file*/
      pcl::PointCloud<pcl::PointXYZI>::Ptr icloud (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::io::loadPCDFile<pcl::PointXYZI> (file_path.c_str(), *icloud);
      
      // seg plane model
      pcloud.plane_model_segmentation(*icloud, icloud);

      // remove all points before adding
      viewer.removeAllPointClouds(); viewer.removeAllShapes();
            
      // color handle
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tot_pcloud_color (icloud, 0,0,0);

      // add scene point clouds
      viewer.addPointCloud<pcl::PointXYZI>(icloud, tot_pcloud_color, file_path.c_str());
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, file_path.c_str());

      // add text
      viewer.addText(video_text, 20,20, 16,0,0,1.0);
      viewer.addText(video_text2, 20,500,25, 139.0/255.0,69.0/255,19/255.0);
      // get ground truth label from tracklets
      trackletextract(viewer, icloud, tracklets, i);

      //save screen shot
      string outputname("./png1/");
      stringstream sstr; sstr << i;
      outputname += sstr.str() + ".png";
      viewer.saveScreenshot(outputname);
      
      viewer.spinOnce();
      
      ++i;
    } else {
      // vector<pcl::visualization::Camera> cam;
      
      // viewer.getCameras(cam);
      // cout << "Cam: " << endl 
      // << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
      // << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
      // << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
      viewer.spinOnce();
    }    
  }
  
  return 0;
}

void trackletextract(pcl::visualization::PCLVisualizer &viewer,
		     pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
		     Tracklets *& tracklets,
		     uint frameid)
{

  pcl::PointCloud<PointXYZI>::Ptr outcloud;

  pcl::CropBox<PointXYZI> clipper;
  clipper.setInputCloud(cloud);
  
  //For each tracklet, extract the points
  for(int i = 0; i < tracklets->numberOfTracklets(); i++){
    if(!tracklets->isActive(i,frameid)){
      continue;
    }

    Tracklets::tTracklet* tracklet = tracklets->getTracklet(i);

    if(true){//objtype.empty() || tracklet->objectType == objtype){

      Tracklets::tPose *pose;
      if(tracklets->getPose(i, frameid, pose)){
	
	outcloud.reset(new pcl::PointCloud<PointXYZI>);
	clipper.setTranslation(Eigen::Vector3f(pose->tx, pose->ty, pose->tz));
	clipper.setRotation(Eigen::Vector3f(pose->rx, pose->ry, pose->rz));
	clipper.setMin(-Eigen::Vector4f(tracklet->l/2, tracklet->w/2, 0, 0));
	clipper.setMax(Eigen::Vector4f(tracklet->l/2, tracklet->w/2, tracklet->h, 0));
	clipper.filter(*outcloud);
	
	// stringstream outfilename; 
	// outfilename << outfile << tracklet->objectType << i << ".pcd";

	if(!outcloud->empty() && outcloud->size() > 100){

	  // objID
	  stringstream ssID; ssID << tracklet->objectType << i;

	  COLOR color;
	  
	  if(ObjId2Color.find( ssID.str() ) != ObjId2Color.end()){

	    color = ObjId2Color[ssID.str()];
	    
	  } else {

	    // map to range image
	    pcl::RangeImage range_image;
	    pcd2RangeImage(outcloud, range_image);

	    // range image to eigen depth and normal matrix
	    cv::Mat depth, normal;
	    RangeImage2cvMatrix(range_image, depth, normal);

	    cout << "Compute HMP: " << range_image.width <<" " << range_image.height << endl;

	    Eigen::VectorXd hmpfea;
	    hmp.computeHMPfromPCD(depth,normal,hmpfea);

	    // feed into online clustering
	    cout << "Online Clustering\n";
	    uint label = str2type(tracklet->objectType);
	    galaxy.nearest_neighbors_insert(hmpfea, global_iter, label,  global_iter, 0.9);
	    //galaxy.V_measure(1, true);

	    auto _G  = galaxy.getGraph();
	    PointXYZI Point = outcloud->at(0);
	    
	    if(_G[global_iter].getType() == Planet::CENTER){	      
	      viewer.addText3D("Request For Label", Point, 0.8,0,0);
	      color = type2color[(OBJ_TYPE)label];	      
	    }
	    else{
	      uint domCenterID = _G[global_iter].getDomCenter(); 
	      uint centerLabel = galaxy.getLabel(domCenterID);
	      color = type2color[(OBJ_TYPE)centerLabel];
	      
	      viewer.addText3D("Fetch Label from Center", Point, 0.8,0,0);
	    }

	    pcl_sleep(1);
	    // insert into map
	    ObjId2Color[ ssID.str() ] = color;
	    
	    global_iter++;
	  }

	  stringstream id;
	  id << frameid << i;
	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
	    tot_pcloud_color(outcloud, color.R,color.G,color.B);
	  
	  viewer.addPointCloud<pcl::PointXYZI>(outcloud, tot_pcloud_color, id.str());
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, id.str());

	  // draw bounding box
	  string boundboxid = id.str();
	  addboundingbox(viewer, outcloud, boundboxid);
	  // cout << "Found "<<outcloud->size() << " points, writing to " << outfilename.str() << endl;
	  // writer.write<PointXYZI> (outfilename.str(), *outcloud, false);
	  
	}
	
      }
    }

  }  
}

void addboundingbox(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, string id)
{

  pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZI min_point_AABB;
  pcl::PointXYZI max_point_AABB;
  pcl::PointXYZI min_point_OBB;
  pcl::PointXYZI max_point_OBB;
  pcl::PointXYZI position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  viewer.addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 0.0, 0.0, 0.0, id.c_str());

  // Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  // Eigen::Quaternionf quat (rotational_matrix_OBB);
  // viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

  // pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  // pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  // pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  // pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));

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

void RangeImage2cvMatrix(pcl::RangeImage &range_image, cv::Mat&depth, cv::Mat&normal)
{

  int width = range_image.width;
  int height = range_image.height;

  float *pixel_value = range_image.getRangesArray();
  depth = cvCreateMat(height, width, CV_64FC1);
  normal = cvCreateMat(height, width, CV_64FC3);
  
  for(uint x = 0; x < width; ++x){
    for(uint y = 0; y < height; ++y){
      
      float var;

      if(pixel_value[x + y*height] < 0.0 || pixel_value[x + y*height] > 10.0e10 || isnan(pixel_value[x + y*height]))
	var = 0.0;
      else{
	var = pixel_value[x + y*height];
      }

      // depth normal
      Eigen::Vector3f normal_vec;
      
      range_image.getNormal(x,y,5,normal_vec);

      depth.at<cv::Vec<double,1> >(y, x) = var;
      normal.at<cv::Vec<double,3> >(y, x) = cv::Vec<double,3>(normal_vec(0), normal_vec(1), normal_vec(2)); 
    }
  }
  
}
