#ifndef __IO_H__
#define __IO_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace onlineclust
{

  using Vec3u8 = cv::Vec<uint8_t, 3>;

  namespace engine
  {
    
    // Note: specially for visualizing data set from
    // http://rgbd-dataset.cs.washington.edu/dataset/
    // convert image to Mat
    // file: file directory/name
    // type: "RGB" or "Depth"
    // matImage: Mat type of image
    void ImgReader(const char *file, const char*type, cv::Mat&matImage);
    void ImgReader(const char *file, const char*type, Eigen::MatrixXd& matImage);
    /// 
    ///
    ///
    void DispImg(cv::Mat const&, const char*);
    /// 
    ///
    ///
    void ImgDim(cv::Mat const&);


    ///
    ///
    ///
    void saveMat(const char dir[],Eigen::MatrixXd &mat, bool if_set_dim);

    ///
    ///
    ///
    std::string readParameter(const char file[], const char param[]);
  }



  
}

#endif
