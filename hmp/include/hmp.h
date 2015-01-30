#ifndef __HMP_H__
#define __HMP_H__

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>


namespace oscl{

  using uchar = unsigned char;
 
  class HMP{

  public:

    HMP(){};
    // Input: configure file directory,
    //        Image type(rgb or rgbd or depth)
    //        feature type(first or second or second+first) 
    HMP(const char*, const char*, const char*);
    ~HMP();

    
    void loadDicts();

    void computeHMP(const char* dir, Eigen::VectorXd &);
    void computeHMP(const char* rgb_dir, const char* depth_dir, Eigen::VectorXd&);

    void im2colstep(Eigen::MatrixXd &rgb, Eigen::MatrixXd &gray, const char* type);

    void max_pooling_layer1(Eigen::MatrixXd &, const char*);

    void max_pooling_layer2(Eigen::VectorXd&, Eigen::VectorXd& );

    void depthtonormal(int topleft[2]);

  private:

    cv::Mat img3C, img1C;
    Eigen::MatrixXd imgDepth, imgNormal[3];

    std::string ImgType;
    std::string FeaType;
    std::string ConfigDir;
    bool if_mask;

    const uint encode_first_pooling = 4;
    const uint pooling[3] = {3,2,1};
    const uint psz1[2] = {5,5};
    const uint psz2[2] = {4,4};
    const uint stepsz1[2] = {1,1};
    const uint stepsz2[2] = {1,1};
    const uint maxsize = 250;
    const uint minsize = 50;
    bool resizetag = false;
    uint spl[2];
    uint L1sz[2], L2sz[2];
    uint sgrid;
    
    const double eps = pow(2,-52);
    
    Eigen::MatrixXd D1rgb, D2rgb;
    Eigen::MatrixXd D1depth, D2depth;
    Eigen::MatrixXd D1normal, D2normal;
    Eigen::MatrixXd D1gray, D2gray;

    Eigen::MatrixXd Gamma1C, Gamma3C;
    
  };

  
  namespace omp{
    /// @param X : observation containing column signals
    /// @param Dct : columns normalized dictionary
    /// @param SPlevel : sparse level constrain
    ///
    /// @output: Sparse code GAMMA such that X ≈ D*GAMMA
    ///
    void Batch_OMP( Eigen::MatrixXd const& X, Eigen::MatrixXd const& Dct, uint SPlevel, Eigen::MatrixXd& Gamma);
 

    // Input: a vector
    // output: maxIdx 
    void maxIdxVec(Eigen::VectorXd const& v, uint &maxIdx); 
  
    // compute w in L*w = G_{I,k} and update L
        // 
    bool updateL(Eigen::MatrixXd& L, Eigen::MatrixXd const& G_Ik, std::vector<uint> const& I, uint k);

    // Input: low-triangular matrix L, rhs b type
    //        L*L^T *x = b(type "LL"),L*x = b(type "L")
    // Output: x
    void LL_solver(Eigen::MatrixXd const& LL, Eigen::VectorXd const& b, uint dimL, const char* type, Eigen::VectorXd &x);

  }
    
    /*
     * helper functions 
     *
     */
    void loadDct(const char* file, int rows, int cols, Eigen::MatrixXd& D);

    void remove_dc(Eigen::MatrixXd &X, const char* type);
    
}


#endif

