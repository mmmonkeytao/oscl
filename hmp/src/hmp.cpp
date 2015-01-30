#include "hmp.h"
#include "io.h"
#include <Eigen/Core>
//#include <cstring>
//#include <stdexcept>

//#include <string>
//#include <cstdlib>
//#include <algorithm>

oscl::HMP::HMP(const char* ConfigDir, const char* ImgType, const char* FeaType)
{
  this->FeaType = std::string(FeaType);
  this->ImgType = std::string(ImgType);
  this->ConfigDir = std::string(ConfigDir);

  loadDicts();

  std::string param = engine::readParameter(ConfigDir, "if_mask");
  
  if_mask = static_cast<bool>(atoi(param.c_str()));

  // get sparsity levels
  param = engine::readParameter(ConfigDir, "spl1");
  spl[0] = static_cast<uint>( atoi(param.c_str()) );
  param = engine::readParameter(ConfigDir, "spl2");
  spl[1] = static_cast<uint>( atoi(param.c_str()) );

  // get resize image flag
  param = engine::readParameter(ConfigDir, "resizetag");
  resizetag = (bool)(atoi(param.c_str()));
}

oscl::HMP::~HMP()
{
  D1rgb = Eigen::MatrixXd();
  D2rgb = Eigen::MatrixXd();
  D1depth = Eigen::MatrixXd();
  D2depth = Eigen::MatrixXd();
  D1normal = Eigen::MatrixXd();
  D2normal = Eigen::MatrixXd();
  D1gray = Eigen::MatrixXd();
  D2gray = Eigen::MatrixXd();	
}


