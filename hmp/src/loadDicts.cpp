#include "hmp.h"
#include "io.h"

void oscl::HMP::loadDicts()
{
  if(!ImgType.compare("rgb")){

    if(!FeaType.compare("first") || !FeaType.compare("second") || !FeaType.compare("second+first")){
            
      std::string str = engine::readParameter(ConfigDir.c_str(), "D1rgb");
      loadDct(str.c_str(), 75, 150, this->D1rgb);
      str = engine::readParameter(ConfigDir.c_str(), "D2rgb");
      loadDct(str.c_str(), 2400, 1000, this->D2rgb);

      str = engine::readParameter(ConfigDir.c_str(), "D1gray");
      loadDct(str.c_str(), 25, 75, this->D1gray);
      str = engine::readParameter(ConfigDir.c_str(), "D2gray");
      loadDct(str.c_str(), 1200, 500, this->D2gray);
      
    } else {
      std::cerr << "Unknow feature type!\n";
    }
    
  } else if(!ImgType.compare("depth")){
    
    if(!FeaType.compare("first") || !FeaType.compare("second") || !FeaType.compare("second+first")){
            
      std::string str = engine::readParameter(ConfigDir.c_str(), "D1depth");
      loadDct(str.c_str(), 25, 75, this->D1depth);
      str = engine::readParameter(ConfigDir.c_str(), "D2depth");
      loadDct(str.c_str(), 1200, 500, this->D2depth);

      str = engine::readParameter(ConfigDir.c_str(), "D1normal");
      loadDct(str.c_str(), 75, 150, this->D1normal);
      str = engine::readParameter(ConfigDir.c_str(), "D2normal");
      loadDct(str.c_str(), 2400, 1000, this->D2normal);
      
    } else {
      std::cerr << "Unknow feature type!\n";
    }

    
  } else if(!ImgType.compare("rgbd")){

    if(!FeaType.compare("first") || !FeaType.compare("second") || !FeaType.compare("second+first")){

      std::string str = engine::readParameter(ConfigDir.c_str(), "D1rgb");
      loadDct(str.c_str(), 75, 150, this->D1rgb);
      str = engine::readParameter(ConfigDir.c_str(), "D2rgb");
      loadDct(str.c_str(), 2400, 1000, this->D2rgb);

      str = engine::readParameter(ConfigDir.c_str(), "D1gray");
      loadDct(str.c_str(), 25, 75, this->D1gray);
      str = engine::readParameter(ConfigDir.c_str(), "D2gray");
      loadDct(str.c_str(), 1200, 500, this->D2gray);

      str = engine::readParameter(ConfigDir.c_str(), "D1depth");
      loadDct(str.c_str(), 25, 75, this->D1depth);
      str = engine::readParameter(ConfigDir.c_str(), "D2depth");
      loadDct(str.c_str(), 1200, 500, this->D2depth);

      str = engine::readParameter(ConfigDir.c_str(), "D1normal");
      loadDct(str.c_str(), 75, 150, this->D1normal);
      str = engine::readParameter(ConfigDir.c_str(), "D2normal");
      loadDct(str.c_str(), 2400, 1000, this->D2normal);
      
    } else {
      std::cerr << "Unknow feature type!\n";
    }
    
  } else {
    std::cerr << "Unknown image type!\n";
  }

}
