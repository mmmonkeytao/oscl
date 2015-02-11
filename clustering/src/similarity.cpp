#include "OnlineStarClustering.h"
#include <cmath>

double oscl::OnlineStarClustering::computeSimilarity(uint id1, uint id2) const
{
  const Eigen::VectorXd &x1 = _data[id1];
  const Eigen::VectorXd &x2 = _data[id2];
  
  if(!_simtype.compare("dot")){ 
    double n1 = x1.norm();
    double n2 = x2.norm();
    if(n1 && n2){
      return x1.dot(x2)/(n1*n2);
    }
    else return 0.0; 
  }
  else if(!_simtype.compare("arccos")){
    double n1 = x1.norm();
    double n2 = x2.norm();

    if(n1 && n2){
      return 1.0 - 2*acos(x1.dot(x2)/(n1*n2))/M_PI;
    }
    else return 0.0;
    
  } else if(!_simtype.compare("Gaussian")){
    return GaussianKernel(id1, id2);
    
  } else if(!_simtype.compare("exp")){
    double n1 = x1.norm();
    double n2 = x2.norm();
    return exp(-(x1/n1 - x2/n2).norm()/(_param));
  }
  else{
    std::cerr << "Unknown similarity type!\n";
  }

  return -1;
}

void oscl::OnlineStarClustering::updateSimilarityMatrix(uint start_idx)
{
  std::cout << "Computing similarity matrix..." << std::endl;

  _similarityMatrix.conservativeResize(_data.size(), _data.size());

  for(uint i=0; i<_data.size(); ++i){
    uint start_j = (start_idx > i) ? start_idx : i;
    for(uint j = start_j; j<_data.size(); ++j){
      _similarityMatrix(i,j) = _similarityMatrix(j,i)
	= computeSimilarity(i,j);
    }
  }

  // std::cout << "done." << std::endl;

  // if(_similarityMatrix.rows() < 25)
  //   std::cout << _similarityMatrix << std::endl;
  // else
  //   std::cout << _similarityMatrix.topLeftCorner<25,25>() << std::endl;
}


