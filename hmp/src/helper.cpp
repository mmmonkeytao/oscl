#include "hmp.h"
#include <fstream>
#include <cmath>


void oscl::loadDct(const char* file, int rows, int cols, Eigen::MatrixXd& D)
{
  std::ifstream input(file);
  D = Eigen::MatrixXd{rows, cols};

  std::cout << "Loading dictionary:\n";

  for(uint j = 0; j < rows; ++j)
    for(uint i = 0; i < cols; ++i)
      input >> D(j,i);

  std::cout << "Loading completes. Dictionary size is: " << D.rows() << "x" << D.cols() << std::endl;
}

void oscl::remove_dc(Eigen::MatrixXd &X, const char* type)
{
  if(!strcmp(type, "column")){
    
    Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(1,X.cols());

    for(uint i = 0; i < X.rows(); ++i){
      mean += X.row(i);
    }
    mean /= (double)X.rows();

    for(uint i = 0; i < X.rows(); ++i){
      X.row(i) -= mean;
    }
  } else {
    std::cerr << "\nUnknown type in OMP::remove_dc.\n";
  }
}

