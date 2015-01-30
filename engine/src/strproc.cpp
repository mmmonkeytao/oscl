#include "proc.h"
#include <string>
#include <sstream>

void oscl::engine::str2doubles(std::string &str, const char*separator, std::vector<double> &v)
{

  // char*  pch = strtok (str.c_str(), separator);

  // while (pch != NULL)
  //   {
  //     pch = strtok (NULL, " ,.-");      
  //   }

  // std::stringstream ss(str);

  // double var;

  // while (ss >> var)
  //   {
  //     vect.push_back(var);

  //     if ( !strcmp(ss.peek(), separator ))
  //       ss.ignore();
  //   }

  size_t pos = 0, next_pos;
  while(1){

    next_pos = str.find_first_of(separator, pos);
        
    double var = atof(str.substr(pos, next_pos).c_str());
    v.push_back(var);

    pos = next_pos + 1;
    
    if(next_pos == std::string::npos || pos >= str.size())
      break;
  } 
  
}
