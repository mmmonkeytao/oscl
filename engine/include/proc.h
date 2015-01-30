#ifndef __PROC_H__
#define __PROC_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "deftype.h"

namespace oscl
{

  namespace engine
  {

    /** process on directories */
    void sub_dir(const char*path_prefix, std::vector<std::string> &subnames);
    void sub_dir_files(const char*path_prefix, const char*suffix, std::vector<std::string> &subnames);


    /** reconstruct image from matrix */
    void reconstructIm(cv::Mat &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat);
    void reconstructIm(Eigen::MatrixXd &im, cv::Size imSize, cv::Size patch, cv::Size stepsz, cv::Mat &mat);

    /** process strings */
    // convert a line of string to store in a eigen3 vector
    void str2doubles(std::string &str, const char*separator, std::vector<double> &v);
  }
  
}

#endif
