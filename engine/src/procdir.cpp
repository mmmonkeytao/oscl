#include "proc.h"
#include <boost/filesystem.hpp>

void oscl::engine::sub_dir(const char*path_prefix, std::vector<std::string> &subnames)
{
  boost::filesystem::path directory(path_prefix);
  boost::filesystem::directory_iterator iter(directory), end;
    
  for(;iter != end; ++iter)
    {
      if(boost::filesystem::is_directory(*iter)) 
        {
  	  std::stringstream temp;
  	  temp << iter->path().filename();
  	  uint len = temp.str().length();
  	  subnames.push_back(temp.str().substr(1,len-2));
        }
    }
}

void oscl::engine::sub_dir_files(const char*path_prefix, const char*suffix, std::vector<std::string> &subnames)
{

  boost::filesystem::path directory(path_prefix);
  boost::filesystem::directory_iterator iter(directory), end;
    
  for(;iter != end; ++iter)
    {
      if(boost::filesystem::is_regular_file(*iter)) 
        {
  	  std::stringstream temp;
  	  temp << iter->path().relative_path();

  	  uint len = temp.str().length();
  	  std::string substr = temp.str().substr(1,len-2);

  	  std::string ssuffix(suffix);

  	  if(!substr.compare(substr.size()-ssuffix.size(),
  			     std::string::npos-1, suffix))
  	    subnames.push_back(substr);
  	}
    }
}
