#ifndef __IO_H__
#define __IO_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Eigen/SparseCore>

namespace oscl
{

  using Vec3u8 = cv::Vec<uint8_t, 3>;

  namespace engine
  {
        
    /**Image Process*/
    void DispImg(cv::Mat const&, const char*);
    void ImgDim(cv::Mat const&);

    // type: "rgb" or "depth"
    // read rgb depth as uint8_t,
    // read depth image as uint16_t
    void ImgReader(const char *file, const char*type, cv::Mat&matImage);
    void ImgReader(const char *file, const char*type, Eigen::MatrixXd& matImage);

    /** read HMP feature file */
    // format: NumOfFeatures feasize \n label \n ...
    void readHMPfile(const char dir[], uint &label, Eigen::SparseMatrix<double> &);
    // format: NumOfFeatures feasize \n label ...
    void readHMPfile(const char dir[], std::vector<uint> labels, Eigen::MatrixXd&);

    /** Matrix output */
    void saveMat(const char dir[],Eigen::MatrixXd &mat, bool if_set_dim);

 
    /** read configuration file*/
    std::string readParameter(const char file[], const char param[]);
  }



  
}

#endif
