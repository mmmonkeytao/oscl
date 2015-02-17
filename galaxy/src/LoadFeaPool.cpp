#include "Galaxy.h"

void oscl::Galaxy::set_fea_pool_null()
{
  _current_fea_pool = std::map<uint, Eigen::VectorXd>();
}

void oscl::Galaxy::load_fea_pool(uint id)
{
  std::stringstream ss;
  ss << id;
  
  std::string fea_path = _fea_path_prefix + '_' + ss.str() + ".dat";  

  std::ifstream ifile(fea_path.c_str());
  VectorXd v{_feaSize};

  for(uint i = 0; i < _feaSize; ++i)
    ifile >> v(i);

  _current_fea_pool[id] = v;
}

void oscl::Galaxy::load_fea_pool(std::set<uint> &centers)
{
  for(auto &x: centers)
    {
      load_fea_pool(x);
    }
}

void oscl::Galaxy::load_fea_pool(std::map<double, uint, std::greater<double> > &max_centers, std::map<double, uint, std::greater<double> >::iterator& max_centers_end)
{
  std::map<double, uint, std::greater<double> >::const_iterator it = max_centers.begin();

  while(it != max_centers_end)
    {

      auto ls = _graph[it->second].getDomSatsList();

      for(auto &x: ls)
	{
	  load_fea_pool(x);
	}

      ++it;
    }
  
}
