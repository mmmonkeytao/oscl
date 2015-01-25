#ifndef __PROC_H__
#define __PROC_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace onlineclust
{

  using Vec3u8 = cv::Vec<uint8_t, 3>;

  namespace engine
  {
    /// image to patch matrix as output 
    /// in which each column represents patch from  
    /// every channel, the order is [r;g;b].
    /// 
    // template<typename T>
    //inline void img2patchMat(Mat const& input, Size patchSize, Size stepSize,Mat &outPatch2dMat);
    void Img2Patch(cv::Mat const& input, cv::Size patchSize, cv::Size stepSize, cv::Mat &patch2dMat);

    void Img2Patch(cv::Mat const& input, cv::Size patchSize, cv::Size stepSize, Eigen::MatrixXd &patch2dMat);

    /// 
    ///
    /// @param im 
    ///
    void reconstructIm(cv::Mat &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat);
    /// 
    ///
    /// @param im 
    /// @param imSize 
    /// @param patch 
    /// @param stepsz 
    /// @param mat 
    ///
    void reconstructIm(Eigen::MatrixXd &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat);

  }



  
}

#endif
