#include "io.h"
#include <opencv2/highgui/highgui.hpp>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <stdexcept>

std::string oscl::engine::readParameter(const char file[], const char param[])
{
  std::ifstream ifile(file);

  while(!ifile.eof()){
    std::string name;
    std::string var;
    ifile >> name >> var;

    if(!name.compare(param)){
      ifile.close();
      return var;
    }
  }

  ifile.close();
  std::cerr << "No parameter has name " << param << std::endl;
  
}

void oscl::engine::saveMat(const char dir[], Eigen::MatrixXd &mat, bool if_set_dim)
{
  std::ofstream ofile(dir);

  uint x = mat.rows();
  uint y = mat.cols();
  
  if(if_set_dim){
    ofile << x << " " << y << std::endl;
  }

  ofile << mat << std::endl;

  ofile.close();
}

void oscl::engine::ImgReader(const char *file, const char*type, cv::Mat&matImage)
{
  std::cout << "\nReading " << type << " Image:\n" 
	    << file << std::endl; 
  
  if(!strcmp(type, "rgb")){
    
    matImage = cv::imread(file,CV_LOAD_IMAGE_COLOR);

    if(matImage.channels()!=3)
      throw std::runtime_error("\nError reading Image.\n");

    matImage.convertTo(matImage, CV_8UC3); //8 bit uint
    
    
  } else if(!strcmp(type, "depth")){

    matImage = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH);

    if(matImage.channels()!=1)
      throw std::runtime_error("\nError reading Image\n");

    matImage.convertTo(matImage, CV_16UC1); // 16bit uint

  }
  else
    throw std::runtime_error("\nUnknown type of image.\n");
}

void oscl::engine::ImgReader(const char *file, const char*type, Eigen::MatrixXd& eigenImage)
{ 
  std::cout << "\nReading " << type << " Image:\n" 
	    << file << std::endl; 

  cv::Mat matImage;
  
  if(!strcmp(type, "rgb")){
    
    matImage = cv::imread(file,CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
    if(matImage.channels()!=3 || !matImage.data)
      throw std::runtime_error("\nError reading Image.\n");

    matImage.convertTo(matImage, CV_8UC3); //8 bit uint

    // convert and normalize from [0 255] to [0 1]
    //cv::Mat dst;
    matImage.convertTo(matImage, CV_64FC3, 1.0, 0);
    //cv::cv2eigen(matImage,eigenImage);
    eigenImage = Eigen::Map<Eigen::MatrixXd,
    			    Eigen::RowMajor,
    			    Eigen::Stride<1,Eigen::Dynamic> >
                  (reinterpret_cast<double*>(matImage.data),
    		   matImage.rows, matImage.cols*3,
    		   Eigen::Stride<1,Eigen::Dynamic>(1, matImage.cols*3));

  } else if(!strcmp(type, "depth")){

    matImage = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH);

    if(matImage.channels()!=1)
      throw std::runtime_error("\nError reading Image\n");

    matImage.convertTo(matImage, CV_16UC1); // 16bit uint

    // convert to double type
    // cv::Mat dst;
    matImage.convertTo(matImage, CV_64FC1, 1.0, 0);
    cv::cv2eigen(matImage, eigenImage);
    // eigenImage = Eigen::Map<Eigen::MatrixXd,
    // 			    Eigen::ColMajor,
    // 			    Eigen::Stride<1,Eigen::Dynamic> >
    //              (reinterpret_cast<double*>(dst.data),
    // 		  dst.rows, dst.cols,
    // 		  Eigen::Stride<1,Eigen::Dynamic>(1, dst.cols));

  }
  else throw std::runtime_error("\nUnknown type of image.\n");
}

void oscl::engine::ImgDim(cv::Mat const&Image)
{ 
  std::cout << "\nImage size:\n" 
	    << Image.rows << "x" << Image.cols << "x" << Image.channels() << std::endl;
}

void oscl::engine::DispImg(cv::Mat const& Image, const char* name)
{
  namedWindow( name, cv::WINDOW_NORMAL ); // Create a window for display.
  imshow( name, Image );                // Show our image inside it.
  cv::waitKey(60000); // Wait for a keystroke in the window
}


