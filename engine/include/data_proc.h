#ifndef __DATAPROC_H__
#define __DATAPROC_H__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Dense>


namespace onlineclust{

  using Vec3u8 = cv::Vec<uint8_t, 3>;

  namespace data_proc{

    
    // Note: specially for visualizing data set from
    // http://rgbd-dataset.cs.washington.edu/dataset/
    // convert image to Mat
    // file: file directory/name
    // type: "RGB" or "Depth"
    // matImage: Mat type of image
    void RGBD_reader(const char*file, char*type, cv::Mat&matImage);
    void RGBD_reader(const char *file, char*type, Eigen::MatrixXd& matImage);
    /// 
    ///
    ///
    void ImgShow(cv::Mat const&, const char*);
    /// 
    ///
    ///
    void ShowImgDim(cv::Mat const&);
  
    /// image to patch matrix as output 
    /// in which each column represents patch from  
    /// every channel, the order is [r;g;b].
    /// 
    // template<typename T>
    //inline void img2patchMat(Mat const& input, Size patchSize, Size stepSize,Mat &outPatch2dMat);
    void im2patchMat(cv::Mat const& input, cv::Size patchSize, cv::Size stepSize, cv::Mat &patch2dMat);

    void im2patchMat(cv::Mat const& input, cv::Size patchSize, cv::Size stepSize, Eigen::MatrixXd &patch2dMat);

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

