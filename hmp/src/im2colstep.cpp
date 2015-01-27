#include "hmp.h"
#include "io.h"
#include <fstream>

void onlineclust::HMP::im2colstep(Eigen::MatrixXd &im1, Eigen::MatrixXd &im2, const char* type)
{
  // for rgb type im1 is rgb, im2 is gray
  // for depth type im1 is normal, im2 is depth

  if(!strcmp(type, "rgbL1")){

    uint offset = psz1[0] * psz1[1];
    uint cols = 0;
    
    for(uint j = 0; j < L1sz[1]; ++j)
      for(uint i = 0; i < L1sz[0]; ++i){

	uint x = i * stepsz1[0];
	uint y = j * stepsz1[1];

	for(uint m = 0; m < psz1[1]; ++m)
	  for(uint l = 0; l < psz1[0]; ++l){

	    uint subx = x+l;
	    uint suby = y+m;

	    double pixelgray = img1C.at<double>(subx, suby);
	    // 0: B, 1: G, 2:R
	    cv::Vec<double,3> v = img3C.at<cv::Vec<double,3> >(subx, suby);

	    im2(l + m*psz1[1], cols) = static_cast<double>(pixelgray);

	    // R
	    im1(l + m*psz1[1], cols) = static_cast<double>(v[2]);
	    // G
	    im1(l + m*psz1[1] + offset, cols) = static_cast<double>(v[1]);
	    // B
	    im1(l + m*psz1[1] + 2*offset, cols) = static_cast<double>(v[0]);	 

	  }

	++cols;		
      }
    
  } else if(!strcmp(type, "rgbL2") || !strcmp(type, "depthL2")) {

    // set layer feature dimention
    L2sz[1] = L1sz[1] + 1 - psz2[1];
    L2sz[0] = L1sz[0] + 1 - psz2[0];

    double threshold = 0.1;

    uint rgbfsz = im1.rows();
    uint depthfsz = im2.rows();

    Eigen::MatrixXd rgbtmp, depthtmp;
    rgbtmp = std::move(im1);
    depthtmp = std::move(im2);

    // reinitialize output features
    im1 = Eigen::MatrixXd{psz2[0]*psz2[1]*rgbfsz, L2sz[0]*L2sz[1]};
    im2 = Eigen::MatrixXd{psz2[0]*psz2[1]*depthfsz, L2sz[0]*L2sz[1]};

    uint offset = psz2[0] * psz2[1];
    uint cols = 0;
    for(uint j = 0; j < L2sz[1]; ++j)
      for(uint i = 0; i < L2sz[0]; ++i){
	
	uint x = i * stepsz2[0];
	uint y = j * stepsz2[1];

	for(uint l = 0; l < psz2[1]; ++l)
	  for(uint m = 0; m < psz2[0]; ++m){

	    uint row = x + m;
	    uint col = y + l;

	    // output 1
	    uint tmp = m+l*psz2[0];
	    for(uint k = 0; k < rgbfsz; ++k){
	      im1(tmp + k*offset,cols) = rgbtmp(k, row + col*L1sz[0]);
	    }
	    // output 2
	    for(uint k = 0; k < depthfsz; ++k){
	      im2(tmp + k*offset,cols) = depthtmp(k, row + col*L1sz[0]);
	    }
	    
	  }	
	++cols;
      }

    //
    Eigen::MatrixXd colNorm_im1 = im1.colwise().norm();
    Eigen::MatrixXd colNorm_im2 = im2.colwise().norm();  
   
    colNorm_im1 = colNorm_im1.cwiseMax(threshold);
    colNorm_im2 = colNorm_im2.cwiseMax(threshold);

    im1.array() = im1.array() / colNorm_im1.colwise().replicate(im1.rows()).array();
    im2.array() = im2.array() / colNorm_im2.colwise().replicate(im2.rows()).array();

  } else if(!strcmp(type, "depthL1")){
    uint offset = psz1[0] * psz1[1];
    uint cols = 0;
 
    for(uint j = 0; j < L1sz[1]; ++j)
      for(uint i = 0; i < L1sz[0]; ++i){

	uint x = i * stepsz1[0];
	uint y = j * stepsz1[1];

	for(uint m = 0; m < psz1[1]; ++m)
	  for(uint l = 0; l < psz1[0]; ++l){

	    uint subx = x+l;
	    uint suby = y+m;

	    im2(l + m*psz1[1], cols) = img1C.at<double>(subx, suby);

	    cv::Vec<double, 3> v = img3C.at<cv::Vec<double,3> >(subx, suby);
	    
	    im1(l + m*psz1[1], cols) = v[0];
	    im1(l + m*psz1[1] + offset, cols) = v[1];
	    im1(l + m*psz1[1] + 2*offset, cols) = v[2];
	    
	  }
	++cols;
      }
    
  } else {
    std::cerr << "Unknow im2colstep type!\n";
  }

}
