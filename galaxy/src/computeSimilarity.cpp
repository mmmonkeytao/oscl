#include "Galaxy.h"
#include <sstream>
#include <fstream>

double oscl::Galaxy::unloaded_computeSimilarity(uint id1, uint id2) const
{
  std::stringstream ss1, ss2;

  ss1 << id1; ss2 << id2;

  std::string fea1_path = _fea_path_prefix + '_' + ss1.str() + ".dat";
  std::string fea2_path = _fea_path_prefix + '_' + ss2.str() + ".dat";

  std::ifstream ifile1(fea1_path.c_str()), ifile2(fea2_path.c_str());

  VectorXd v1{_feaSize}, v2{_feaSize};

  for(uint i = 0; i < _feaSize; ++i)
    {
      ifile1 >> v1(i);
      ifile2 >> v2(i);
    }

  return 1.0 / (v1 - v2).cwiseAbs().sum();
  
}

double oscl::Galaxy::loaded_computeSimilarity(uint id1, uint id2)
{
  return 1.0 / (_current_fea_pool[id1] - _current_fea_pool[id2]).cwiseAbs().sum();
}

double oscl::Galaxy::computeSimilarity(uint id1, uint id2)
{
  return 1.0 / (_dataXXL[id1] - _dataXXL[id2]).cwiseAbs().sum();
}
