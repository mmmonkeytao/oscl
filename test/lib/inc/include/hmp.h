#ifndef __HMP_H__
#define __HMP_H__

#include <iostream>
#include <Eigen/Dense>
#include <vector>


namespace onlineclust{

  using uchar = unsigned char;
  // first: width, second: height
  using matSize = std::pair<uint, uint>;
  //using Mat3D = Matrix<VectorXd, Dynamic, Dynamic >;
  
  class HMP{

  public:
    /// 
    ///
    /// @param x 
    /// @param type 
    /// @param SPlevel 
    /// @param fea 
    ///
    /// second
    void hmp_core(Eigen::MatrixXd &x, const char *type, uint SPlevel[2], Eigen::MatrixXd &fea);
    // first + second
    void hmp_core_mode1(Eigen::MatrixXd &x, const char *type, uint SPlevel[2], Eigen::MatrixXd &fea);
    // first
    void hmp_core_mode2(Eigen::MatrixXd &x, const char *type, uint SPlevel[2], Eigen::MatrixXd &fea);
    /// 
    ///
    /// @param D 
    /// @param X 
    /// @param splevel 
    /// @param SPcodes 
    ///
    void MaxPool_layer1_mode1(Eigen::MatrixXd const&Gamma, matSize const&patchsz, matSize const&realsz, Eigen::MatrixXd &omp_pooling, uint sz[2]);

    void max_pooling(Eigen::MatrixXd& mat, uint sz[2], uint pool[2]);
    /// 
    ///
    /// @param fea 
    /// @param feaSz 
    /// @param pool 
    ///
    //    void MaxPool_layer2(Eigen::MatrixXd &fea, matSize const&feaSz, uint pool[3]);
    void MaxPool_layer2(Eigen::MatrixXd &ifea, uint sz[2], uint pool[3], Eigen::MatrixXd &ofea);

    /// 
    ///
    /// @param im 
    /// @param type 
    /// @param rsz 
    /// @param patchMat 
    ///
    void mat2patch(Eigen::MatrixXd const& im, const char*type, matSize const& rsz, Eigen::MatrixXd &patchMat);
    // x,y,z
    void mat2col(Eigen::MatrixXd const& mat, uint dim[3], uint patchsz[2], uint stepsz[3], Eigen::MatrixXd &matcol, uint sz[2]);
    /// 
    ///
    /// @param flayer1 
    /// @param flayer2 
    /// @param type 
    ///
    void load2Dcts(const char* layer1, const char*layer2, const char* type);
    

    struct patchsz{
      const uint width = 5;
      const uint height = 5;
    }patchsz;
    
    struct stepsz{
      const uint width1 = 1;
      const uint height1 = 1;
      const uint width2 = 1;
      const uint height2 = 1;
    }stepsz;

  private:
    const double eps = pow(2,-52);
    Eigen::MatrixXd D1rgb;
    Eigen::MatrixXd D2rgb;
    Eigen::MatrixXd D1depth;
    Eigen::MatrixXd D2depth;
    Eigen::MatrixXd D1normal;
    Eigen::MatrixXd D2normal;
    Eigen::MatrixXd D1gray;
    Eigen::MatrixXd D2gray;
    
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
    /// @param file 
    /// @param rows 
    /// @param cols 
    /// @param D 

    ///
    void loadDct(const char* file, int rows, int cols, Eigen::MatrixXd& D);

    /// 
    ///
    /// @param X 
    /// @param type 
    ///
    void remove_dc(Eigen::MatrixXd &X, const char* type);
    
    
}


#endif

