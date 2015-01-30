#include "hmp.h"
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <fstream>


void oscl::omp::Batch_OMP( Eigen::MatrixXd const& X, Eigen::MatrixXd const& D, uint SPlevel, Eigen::MatrixXd &Gamma )
{
  using namespace Eigen;
  using namespace std;
  
  uint Xrow = X.rows(), Xcol = X.cols(), Drow = D.rows(), Dcol = D.cols();

  if( !Xrow || !Xcol || !Drow || !Dcol || Xrow != Drow)
    throw std::runtime_error("\nInput parameters are wrong in OMP::Bach_OMP.\n");

  // compute matrix G = D' D, G: Dcol by Dcol
  MatrixXd G = D.transpose() * D;
  
  // compute apha^0 = D' x, alpha0: Dcol by Xcol
  MatrixXd alpha0 = D.transpose() * X;   
  
  // initialization 
  Gamma = MatrixXd::Zero(Dcol, Xcol);
  
  // iteration no. of obersations in X
  for(uint i = 0; i < Xcol; ++i){
    // I stores ordered max_k
    vector<uint> I;    
    // initialize L
    MatrixXd L = MatrixXd::Zero(SPlevel, SPlevel);
    L(0,0) = 1; 
    // initialize vector to check if same atom is selected
    vector<bool> selected(Dcol,false);
 
    VectorXd a_I{SPlevel}, r_I{SPlevel};

    VectorXd alpha = alpha0.col(i);
    // 
    MatrixXd G_I{Dcol, SPlevel};
    
    // inner loop for no. of atoms in D
    for(uint j = 0; j < SPlevel; ++j){

      uint k; 
      maxIdxVec(alpha, k);
      
      double alpha_k = alpha(k);

      // select same atom or dependent atom
      if(selected[k] || alpha_k*alpha_k < 1e-14) break;

      //if(j > 0 && !oscl::omp::updateL(L, G, I, k)) break;
      if(j > 0 && !oscl::omp::updateL(L, G, I, k)){
	  break;
      } 

      selected[k] = true;
      I.push_back(k);
            
      a_I(j) = alpha0(k,i);
      LL_solver(L, a_I, j+1,"LL", r_I);

      G_I.col(j) = G.col(I[j]);

      alpha = alpha0.col(i) - G_I.leftCols(j+1) * r_I.head(j+1);

    }

    uint count = 0;
    for(auto &x: I) 
      Gamma(x,i) = r_I(count++);
    
  }
}

bool oscl::omp::updateL( Eigen::MatrixXd & L, Eigen::MatrixXd const& G, std::vector<uint> const& I, uint k)
{
  uint dim = I.size();
  Eigen::VectorXd g{dim};

  for(uint i = 0; i < dim; ++i)
    g(i) = G(k, I[i]);

  // solve for w linear system L * w = g using Cholesky decomposition
  Eigen::VectorXd omega{dim};
  LL_solver(L, g, dim,"L", omega);
  
  // update L = [ L 0; w^T sqrt(1 - w'w)]
  L.block(dim,0,1,dim) = omega.transpose();
  
  double sum = 1 - omega.dot(omega);

  if(sum <= 1e-14)
    return false;
  
  L(dim, dim) = sqrt(sum);
  
  return true;
}

void oscl::omp::LL_solver(Eigen::MatrixXd const& L, Eigen::VectorXd const& b, uint dimL, const char* type, Eigen::VectorXd& x)
{
  Eigen::VectorXd w{dimL}, b1{b.head(dimL)};
   
  // L^T * w = b/L
  for(uint i = 0; i < dimL; ++i){

    for(uint j = 0; j < i; ++j)
      b1(i) -= w(j)*L(i,j);
       
    w(i) = (double)b1(i)/L(i,i);
  }

  if(!strcmp("L", type)) { x = w; return; }

  // w = x/L^T
  for(int i = dimL-1; i >= 0; --i){

    for(int j = dimL-1; j > i; --j)
      w(i) -= x(j)*L(j,i);
        
    x(i) = (double)w(i)/L(i,i);
  }
}

void oscl::omp::maxIdxVec(Eigen::VectorXd const& v, uint &maxIdx)
{
  double max = -1;
  
  for(uint i = 0; i < v.rows(); i++){
    if(fabs(v.coeff(i)) > max){
      max = fabs(v.coeff(i));
      maxIdx = i;
    }
  }
}

