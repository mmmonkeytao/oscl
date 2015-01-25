#include "data_proc.h"
#include <opencv2/highgui/highgui.hpp>
#include <cstdint>
#include <cmath>

void onlineclust::data_proc::im2patchMat(cv::Mat const& input, cv::Size psz, cv::Size stpsz,cv::Mat &patch2dMat)
{
  
  int nPatchx = ceil((float)(input.cols - psz.width)/(float)stpsz.width);
  int nPatchy = ceil((float)(input.rows - psz.height)/(float)stpsz.height);

  patch2dMat = cv::Mat{psz.width * psz.height * input.channels(),
		   nPatchx * nPatchy, CV_64FC1};
  
  uint cols = 0, srow, scol;
  int npx = psz.height * psz.width;

  for(int j = 0; j < nPatchy; ++j){
    for(int i = 0; i < nPatchx; ++i){
      scol = i*stpsz.width;
      srow = j*stpsz.height;

      //getRectSubPix(input, psz,{static_cast<float>(scol),static_cast<float>(srow)},patch,input.type());
      // copy to output matrix patch2dMat, order r,g,b
	for(int m = 0; m < psz.height; ++m)
	  for(int l = 0; l < psz.width; ++l){	     
	    onlineclust::Vec3u8 v = input.at<onlineclust::Vec3u8>(srow + m, scol + l);
	    
	    patch2dMat.at<double>(l+m*psz.width,cols) = (double)v[2]/255.0;
	    patch2dMat.at<double>(l+m*psz.width + npx, cols) = (double)v[1]/255.0;
	    patch2dMat.at<double>(l+m*psz.width + 2*npx, cols) = (double)v[0]/255.0;
	  }
      ++cols;
    }
  }      
}

void onlineclust::data_proc::im2patchMat(cv::Mat const& input, cv::Size psz, cv::Size stpsz,Eigen::MatrixXd &patch2dMat)
{
  
  int nPatchx = ceil((float)(input.cols - psz.width)/(float)stpsz.width);
  int nPatchy = ceil((float)(input.rows - psz.height)/(float)stpsz.height);

  patch2dMat = Eigen::MatrixXd{psz.width * psz.height * input.channels(),
		   nPatchx * nPatchy};
 
  uint cols = 0, srow, scol;
  int npx = psz.height * psz.width;

  for(int j = 0; j < nPatchy; ++j){
    for(int i = 0; i < nPatchx; ++i){
      scol = i*stpsz.width;
      srow = j*stpsz.height;

      // copy to output matrix patch2dMat, order r,g,b
	for(int m = 0; m < psz.height; ++m)
	  for(int l = 0; l < psz.width; ++l){

	    if(input.channels() == 3){
	      onlineclust::Vec3u8 v = input.at<onlineclust::Vec3u8>(srow + m, scol + l); // 
	      patch2dMat(l+m*psz.width,cols) = (double)v[2]/255.0;
	      patch2dMat(l+m*psz.width + npx, cols) = (double)v[1]/255.0;
	      patch2dMat(l+m*psz.width + 2*npx, cols) = (double)v[0]/255.0;
	    } else {
	      patch2dMat(l+m*psz.width,cols) = (double)input.at<uint16_t>(srow + m, scol + l) / (double)pow(2,16);
	    }
	    
	  }
      ++cols;
    }
  }      
}

void onlineclust::data_proc::reconstructIm(cv::Mat &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat)
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
	  onlineclust::Vec3u8 v;

	  v[2] = static_cast<uint8_t>(im.at<double>(k+l*pszw, pth) * 255);
	  v[1] = static_cast<uint8_t>(im.at<double>(k+l*pszw+psz, pth) * 255);
	  v[0] = static_cast<uint8_t>(im.at<double>(k+l*pszw+psz*2, pth) * 255);

	  mat.at<onlineclust::Vec3u8>(sh+l, sw+k) = v;

	}
    }
}

void onlineclust::data_proc::reconstructIm(Eigen::MatrixXd &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat)
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
	  	    onlineclust::Vec3u8 v;
	  v[2] = static_cast<uint8_t>(im(k+l*pszw, pth) * 255.0);
	  v[1] = static_cast<uint8_t>(im(k+l*pszw+psz, pth) * 255.0);
	  v[0] = static_cast<uint8_t>(im(k+l*pszw+psz*2, pth) * 255.0);
	  mat.at<onlineclust::Vec3u8>(sh+l,sw+k) = v;

	} //break;
 
    }
}

void onlineclust::data_proc::RGBD_reader(const char *file, char*type, cv::Mat&matImage)
{
  std::cout << "\nReading " << type << " Image:\n" 
	    << file << std::endl; 
  
  if(!strcmp(type, "RGB")){
    
    matImage = cv::imread(file,CV_LOAD_IMAGE_COLOR);
    if(matImage.channels()!=3 || !matImage.data)
      throw std::runtime_error("\nError reading Image.\n");

    matImage.convertTo(matImage, CV_8U); //8 bit uint

  } else if(!strcmp(type, "Depth")){

    matImage = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH);

    if(matImage.channels()!=1)
      throw std::runtime_error("\nError reading Image\n");

    matImage.convertTo(matImage, CV_16U); // 16bit uint
  }
  else throw std::runtime_error("\nUnknown type of image.\n");
}

void onlineclust::data_proc::RGBD_reader(const char *file, char*type, Eigen::MatrixXd& eigenImage)
{ 
  std::cout << "\nReading " << type << " Image:\n" 
	    << file << std::endl; 

  cv::Mat matImage;
  
  if(!strcmp(type, "RGB")){
    
    matImage = cv::imread(file,CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
    if(matImage.channels()!=3 || !matImage.data)
      throw std::runtime_error("\nError reading Image.\n");

    matImage.convertTo(matImage, CV_8UC3); //8 bit uint

    // convert and normalize from [0 255] to [0 1]
    cv::Mat dst;
    matImage.convertTo(dst, CV_64FC3, 1.0/255.0, 0);
    
    eigenImage = Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor, Eigen::Stride<1,Eigen::Dynamic> >(reinterpret_cast<double*>(dst.data), dst.rows, dst.cols*3, Eigen::Stride<1,Eigen::Dynamic>(1, dst.cols*3));

  } else if(!strcmp(type, "Depth")){

    matImage = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH);

    if(matImage.channels()!=1)
      throw std::runtime_error("\nError reading Image\n");

    matImage.convertTo(matImage, CV_16UC3); // 16bit uint

    // convert and normalize from [0 255] to [0 1]
    cv::Mat dst;
    matImage.convertTo(dst, CV_64FC3, 1.0/65536.0, 0);
    
    eigenImage = Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor, Eigen::Stride<1,Eigen::Dynamic> >(reinterpret_cast<double*>(dst.data), dst.rows, dst.cols, Eigen::Stride<1,Eigen::Dynamic>(1, dst.cols));

  }
  else throw std::runtime_error("\nUnknown type of image.\n");
}

void onlineclust::data_proc::ShowImgDim(cv::Mat const&Image)
{ 
  std::cout << "\nImage size:\n" 
	    << Image.rows << "x" << Image.cols << "x" << Image.channels() << std::endl;
}

void onlineclust::data_proc::ImgShow(cv::Mat const& Image, const char* name)
{
  namedWindow( name, cv::WINDOW_NORMAL ); // Create a window for display.
  imshow( name, Image );                // Show our image inside it.
    //waitKey(600); // Wait for a keystroke in the window
}


