#include "proc.h"
#include <opencv2/highgui/highgui.hpp>
#include <cstdint>
#include <cmath>

void oscl::engine::reconstructIm(cv::Mat &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat)
{
  int imh = imSize.height, imw = imSize.width;
  int pszh = patch.height, pszw = patch.width;
  int pnw = ceil((double)(imw - pszw)/ (double)stepsz.width);
  int pnh = ceil((double)(imh - pszh)/ (double)stepsz.height);
  
  mat = cv::Mat{pszh*pnh, pszw*pnw, CV_8UC3};

  int psz = pszh * pszw;

  for(int j = 0; j < pnh; ++j)
    for(int i = 0; i < pnw; ++i){
      int sw = i * pszw;
      int sh = j * pszh;
      int pth = i + j*pnw;
  
      for(int l = 0; l < pszh; ++l)
	for(int k = 0; k < pszw; ++k){
	  oscl::Vec3u8 v;

	  v[2] = static_cast<uint8_t>(im.at<double>(k+l*pszw, pth) * 255);
	  v[1] = static_cast<uint8_t>(im.at<double>(k+l*pszw+psz, pth) * 255);
	  v[0] = static_cast<uint8_t>(im.at<double>(k+l*pszw+psz*2, pth) * 255);

	  mat.at<oscl::Vec3u8>(sh+l, sw+k) = v;

	}
    }
}

void oscl::engine::reconstructIm(Eigen::MatrixXd &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat)
{
  int imh = imSize.height, imw = imSize.width;
  int pszh = patch.height, pszw = patch.width;
  int pnw = ceil((double)(imw - pszw)/ (double)stepsz.width);
  int pnh = ceil((double)(imh - pszh)/ (double)stepsz.height);
  
  mat = cv::Mat{pszh*pnh, pszw*pnw, CV_8UC3};

  int psz = pszh * pszw;

  for(int j = 0; j < pnh; ++j)
    for(int i = 0; i < pnw; ++i){
      int sw = i * pszw;
      int sh = j * pszh;
      int pth = i + j*pnw;
  
      for(int l = 0; l < pszh; ++l)
	for(int k = 0; k < pszw; ++k){	  
	  	    oscl::Vec3u8 v;
	  v[2] = static_cast<uint8_t>(im(k+l*pszw, pth) * 255.0);
	  v[1] = static_cast<uint8_t>(im(k+l*pszw+psz, pth) * 255.0);
	  v[0] = static_cast<uint8_t>(im(k+l*pszw+psz*2, pth) * 255.0);
	  mat.at<oscl::Vec3u8>(sh+l,sw+k) = v;

	}
 
    }
}
