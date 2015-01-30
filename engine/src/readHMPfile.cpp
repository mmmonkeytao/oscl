#include "io.h"
#include "proc.h"
#include <string>

void oscl::engine::readHMPfile(const char dir[], uint &label, Eigen::SparseMatrix<double> &mat)
{
  std::ifstream ifile(dir);
  uint fea_num;
  uint fea_size;

  std::string line;
  getline(ifile, line);

  size_t loc = line.find_first_of(" ");
  fea_num = atoi(line.substr(0, loc).c_str());
  fea_size = atoi(line.substr(loc+1, line.size()).c_str());
  
  getline(ifile, line);
  label = atoi(line.c_str());
     
  // init data matrix, features are stored colwise
  mat = Eigen::SparseMatrix<double>(fea_size, fea_num);

  uint col = 0;
  
  while(!ifile.eof()){

    getline(ifile, line);
  
    if(line.size() > 0){
      
      size_t pos = 0, next_pos;
      do{
	next_pos = line.find_first_of(' ', pos);
	double loc = atof(line.substr(pos, next_pos).c_str());
	pos = next_pos + 1;
	next_pos = line.find_first_of(' ', pos);
	double var = atof(line.substr(pos, next_pos).c_str());
	pos = next_pos + 1;
	
	mat.insert((uint)loc, col) = var;

      }while(next_pos < line.size()-1);
      
      ++col;
    }
  }
}

void oscl::engine::readHMPfile(const char dir[], std::vector<uint> labels, Eigen::MatrixXd &mat)
{
  std::ifstream ifile(dir);
  uint fea_num;
  uint fea_size;

  std::string line;
  getline(ifile, line);

  size_t loc = line.find_first_of(" ");
  fea_num = atoi(line.substr(0, loc).c_str());
  fea_size = atoi(line.substr(loc+1, line.size()).c_str());
       
  // init data matrix, features are stored colwise
  mat = Eigen::MatrixXd::Zero(fea_size, fea_num);

  uint col = 0;
  
  while(!ifile.eof()){

    getline(ifile, line);
    
    if(line.size() > 0){
      
      std::vector<double> v;

      size_t pos = 0, next_pos;
      next_pos = line.find_first_of(' ', pos);
      pos = next_pos + 1;
      labels.push_back(atoi(line.substr(0, next_pos).c_str()));

      do{
	next_pos = line.find_first_of(' ', pos);
	double loc = atof(line.substr(pos, next_pos).c_str());
	pos = next_pos + 1;
	next_pos = line.find_first_of(' ', pos);
	double var = atof(line.substr(pos, next_pos).c_str());

	mat((uint)loc, col) = var;

      }while(next_pos < line.size()-1);
      
      ++col;
    }
  }  
}
