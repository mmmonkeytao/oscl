#include "hmp.h"
#include "io.h"
#include <fstream>

void onlineclust::HMP::computeHMP(const char* rgb_dir, const char* depth_dir, Eigen::VectorXd &feaHMP)
{

  Eigen::VectorXd rgbfea, depthfea;

  // set current type to "rgb"
  ImgType = std::string("rgb");
  computeHMP(rgb_dir, rgbfea);

  // set current type to "depth"
  ImgType = std::string("depth");
  computeHMP(depth_dir, depthfea);

  feaHMP = Eigen::VectorXd{rgbfea.size() + depthfea.size()};
  feaHMP.head(depthfea.size()) = depthfea;
  feaHMP.tail(rgbfea.size()) = rgbfea;
}

void onlineclust::HMP::computeHMP(const char* dir, Eigen::VectorXd &feaHMP)
{
  using namespace Eigen;
  using namespace cv;
  using namespace std;
  
  if(!ImgType.compare("rgb")){

    // read Image
    engine::ImgReader(dir, ImgType.c_str(), img3C);

    uint rows, cols;
    rows = img3C.rows;
    cols = img3C.cols;

    // rgb to gray
    img1C = cv::Mat(rows, cols, CV_8UC1);
    cvtColor(img3C, img1C, CV_BGR2GRAY);

    // compute first level patch image size
    // 0: row, 1: col
    L1sz[1] = ceil((float)(cols - (uint)(psz1[1]/2) * 2)/(float)stepsz1[1]);
    L1sz[0] = ceil((float)(rows - (uint)(psz1[0]/2) * 2)/(float)stepsz1[0]);

    // convert image to patch matrix
    MatrixXd rgbPatch{3*psz1[0]*psz1[1], L1sz[0]*L1sz[1]},
             grayPatch{psz1[0]*psz1[1], L1sz[0]*L1sz[1]};
    im2colstep(rgbPatch, grayPatch, "rgbL1");

    // remove dc from column
    remove_dc(rgbPatch, "column");
    remove_dc(grayPatch, "column");

    // first layer coding
    omp::Batch_OMP(rgbPatch, D1rgb, spl[0], Gamma3C);
    omp::Batch_OMP(grayPatch, D1gray, spl[0], Gamma1C);

    // convert to absolute value
    Gamma3C = Gamma3C.cwiseAbs();
    Gamma1C = Gamma1C.cwiseAbs();

    // max pooling layer1
    MatrixXd rgb_pooling_l1, gray_pooling_l1; 
    max_pooling_layer1(rgb_pooling_l1, "3C");
    max_pooling_layer1(gray_pooling_l1,"1C");

    // update L1sz
    L1sz[0] = floor((float)L1sz[0]/(float)encode_first_pooling);
    L1sz[1] = floor((float)L1sz[1]/(float)encode_first_pooling);

    // free Gamma memory
    Gamma3C = MatrixXd();
    Gamma1C = MatrixXd();

    // patch sample layer2
    MatrixXd rgb_omp_fea, gray_omp_fea;
    rgb_omp_fea = rgb_pooling_l1;
    gray_omp_fea = gray_pooling_l1;
    im2colstep(rgb_omp_fea, gray_omp_fea, "rgbL2");
    Gamma3C = std::move(rgb_omp_fea);
    Gamma1C = std::move(gray_omp_fea);

    // check if first layer fea will be pooled
    sgrid = pow(pooling[0],2) + pow(pooling[1],2) + pow(pooling[2],2);
    VectorXd rgb_first, gray_first;

    if(!FeaType.compare("first") || !FeaType.compare("second+first")){
      rgb_first = VectorXd{sgrid*rgb_omp_fea.rows()};
      gray_first = VectorXd{sgrid*gray_omp_fea.rows()};

      max_pooling_layer2(rgb_first, gray_first);

      rgb_first /= sqrt(rgb_first.squaredNorm() + eps);
      gray_first /= sqrt(gray_first.squaredNorm() + eps);
      
      if(!FeaType.compare("first")){
	feaHMP = VectorXd{rgb_first.size() + gray_first.size()};
	feaHMP.head(gray_first.size()) = gray_first;
	feaHMP.segment(gray_first.size(), rgb_first.size()) = rgb_first;
	feaHMP.normalize();
	return;
      }
    }
    // coding layer2
    omp::Batch_OMP(rgb_omp_fea, D2rgb, spl[1], Gamma3C);
    omp::Batch_OMP(gray_omp_fea, D2gray, spl[1], Gamma1C);

    // convert to absolute value
    Gamma3C = Gamma3C.cwiseAbs();
    Gamma1C = Gamma1C.cwiseAbs();

    // pooling layer2 to feature
    VectorXd rgb_second{sgrid*Gamma3C.rows()};
    VectorXd gray_second{sgrid*Gamma1C.rows()};

    max_pooling_layer2(rgb_second, gray_second);

    rgb_second /= sqrt(rgb_second.squaredNorm() + eps);
    gray_second /= sqrt(gray_second.squaredNorm() + eps);

    if(!FeaType.compare("second")){
      	feaHMP = VectorXd{rgb_second.size() + gray_second.size()};
	feaHMP.head(gray_second.size()) = gray_second;
	feaHMP.segment(gray_second.size(), rgb_second.size()) = rgb_second;
	feaHMP.normalize();
	return;
    }

    if(!FeaType.compare("second+first")){
      	feaHMP = VectorXd{rgb_second.size() + gray_second.size() + rgb_first.size() + gray_first.size()};
	feaHMP.head(gray_second.size()) = gray_second;
	feaHMP.segment(gray_second.size(), gray_first.size()) = gray_first;
	feaHMP.segment(gray_second.size()+gray_first.size(), rgb_second.size()) = rgb_second;
	feaHMP.tail(rgb_first.size()) = rgb_first;
	feaHMP.normalize();
	return;      
    }

  } else if(!ImgType.compare("depth")){

    // read Depth Image
    engine::ImgReader(dir, ImgType.c_str(), imgDepth);

    uint rows, cols;
    rows = imgDepth.rows();
    cols = imgDepth.cols();

    // read location file
    string s(dir);
    s = s.substr(0, s.find_last_of("_")+1);
    s = s + "loc.txt";
    
    // depth to normal
    ifstream ifile(s.c_str());    
    ifile >> s;
    ifile.close();

    //
    int topleft[2];
    topleft[0] = atoi(s.substr(s.find_first_of(",")+1, s.size()).c_str());
    topleft[1] = atoi(s.substr(0, s.find_first_of(",")).c_str());    
    depthtonormal(topleft);

    // process imgDepth
    double threshold = 1200.0;
    if(if_mask){
      imgDepth.cwiseMin(threshold);

      for(uint j = 0; j < imgDepth.cols(); ++j)
	for(uint i = 0; i < imgDepth.rows(); ++i)
	  if(imgDepth(i,j) == 0.0)
	    imgDepth(i,j) = 400.0;
      imgDepth.array() = imgDepth.array() / threshold;
      
    } else {      
      imgDepth.cwiseMin(threshold);
      imgDepth.array() /= threshold;
    }

    // compute first level patch image size
    // 0: row, 1: col
    L1sz[1] = ceil((float)(cols - (uint)(psz1[1]/2) * 2)/(float)stepsz1[1]);
    L1sz[0] = ceil((float)(rows - (uint)(psz1[0]/2) * 2)/(float)stepsz1[0]);

    // convert image to patch matrix
    MatrixXd normalPatch{3*psz1[0]*psz1[1], L1sz[0]*L1sz[1]},depthPatch{psz1[0]*psz1[1], L1sz[0]*L1sz[1]};	     
    im2colstep(normalPatch, depthPatch, "depthL1");

    // remove dc from column
    remove_dc(normalPatch, "column");
    remove_dc(depthPatch, "column");

    // first layer coding
    omp::Batch_OMP(normalPatch, D1normal, spl[0], Gamma3C);
    omp::Batch_OMP(depthPatch, D1depth, spl[0], Gamma1C);
    
    // convert to absolute value
    Gamma3C = Gamma3C.cwiseAbs();
    Gamma1C = Gamma1C.cwiseAbs();

    // max pooling layer1
    MatrixXd normal_pooling_l1, depth_pooling_l1; 
    max_pooling_layer1(normal_pooling_l1, "3C");
    max_pooling_layer1(depth_pooling_l1,"1C");
  
    // update L1sz
    L1sz[0] = floor((float)L1sz[0]/(float)encode_first_pooling);
    L1sz[1] = floor((float)L1sz[1]/(float)encode_first_pooling);

    // free Gamma memory
    Gamma3C = MatrixXd();
    Gamma1C = MatrixXd();

    // patch sample layer2
    MatrixXd normal_omp_fea, depth_omp_fea;
    normal_omp_fea = normal_pooling_l1;
    depth_omp_fea = depth_pooling_l1;
    im2colstep(normal_omp_fea, depth_omp_fea, "depthL2");
    Gamma3C = std::move(normal_omp_fea);
    Gamma1C = std::move(depth_omp_fea);

    // check if first layer fea will be pooled
    sgrid = pow(pooling[0],2) + pow(pooling[1],2) + pow(pooling[2],2);
    VectorXd normal_first, depth_first;

    if(!FeaType.compare("first") || !FeaType.compare("second+first")){
      normal_first = VectorXd{sgrid*normal_omp_fea.rows()};
      depth_first = VectorXd{sgrid*depth_omp_fea.rows()};

      max_pooling_layer2(normal_first, depth_first);

      normal_first /= sqrt(normal_first.squaredNorm() + eps);
      depth_first /= sqrt(depth_first.squaredNorm() + eps);
      
      if(!FeaType.compare("first")){
	feaHMP = VectorXd{normal_first.size() + depth_first.size()};
	feaHMP.head(depth_first.size()) = depth_first;
	feaHMP.segment(depth_first.size(), normal_first.size()) = normal_first;
	feaHMP.normalize();
	return;
      }
    }
    // coding layer2
    omp::Batch_OMP(normal_omp_fea, D2normal, spl[1], Gamma3C);
    omp::Batch_OMP(depth_omp_fea, D2depth, spl[1], Gamma1C);

    // convert to absolute value
    Gamma3C = Gamma3C.cwiseAbs();
    Gamma1C = Gamma1C.cwiseAbs();

    // pooling layer2 to feature
    VectorXd normal_second{sgrid*Gamma3C.rows()};
    VectorXd depth_second{sgrid*Gamma1C.rows()};

    max_pooling_layer2(normal_second, depth_second);

    normal_second /= sqrt(normal_second.squaredNorm() + eps);
    depth_second /= sqrt(depth_second.squaredNorm() + eps);

    if(!FeaType.compare("second")){
      	feaHMP = VectorXd{normal_second.size() + depth_second.size()};
	feaHMP.head(depth_second.size()) = depth_second;
	feaHMP.segment(depth_second.size(), normal_second.size()) = normal_second;
	feaHMP.normalize();
	return;
    }

    if(!FeaType.compare("second+first")){
      	feaHMP = VectorXd{normal_second.size() + depth_second.size() + normal_first.size() + depth_first.size()};
	feaHMP.head(depth_second.size()) = depth_second;
	feaHMP.segment(depth_second.size(), depth_first.size()) = depth_first;
	feaHMP.segment(depth_second.size()+depth_first.size(), normal_second.size()) = normal_second;
	feaHMP.tail(normal_first.size()) = normal_first;
	feaHMP.normalize();
	return;      
    }

  }  
}

