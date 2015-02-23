#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include "proc.h"

using namespace std::chrono;
using namespace std;
using namespace oscl::engine;
using namespace Eigen;

uint NUM_CLASS;
uint FEA_SIZE;

vector<int> labels;
vector<VectorXd> data;
auto labelist = map<int, vector<uint> >();
string data_prefix;
vector<VectorXd> sim;

void init_data();
void feed_sim(const char*);
double compute_similarity(VectorXd &x1, VectorXd &x2, const char* type);
void save_sim(const char*);

int main(int argc, char** argv){

  if(argc != 6){
    cerr << "Usage: <exec> <prefix> <save_dir> <sim_type> <feature size> <number of class>\n";
    return -1;
  }

  data_prefix = string(argv[1]);
  NUM_CLASS = atoi(argv[5]);
  FEA_SIZE = atoi(argv[4]);
  
  init_data();

  // feeding similarity information
  cout << "feeding sim.\n";

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  feed_sim(argv[3]);
  cout << "saving file.\n";
  save_sim(argv[2]);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  cout << "Time spent: " << duration_cast<seconds>(t2-t1).count() << endl;
  return 0;
}


void init_data()
{  
  uint data_idx = 0;

  vector<string> filenames;
  sub_dir_files(data_prefix.c_str(), "hmp.dat", filenames);
  
  for(uint file_counter = 0; file_counter < filenames.size(); ++file_counter)
    {
      size_t pos = filenames[file_counter].find_last_of('_');
      string basename = filenames[file_counter].substr(0, pos+1); 
      
      string label_path =  basename + "label.dat";

      //cout << label_path << "    " << filenames[file_counter] << endl;
      
      ifstream ifdata(filenames[file_counter].c_str()), iflabel(label_path.c_str());

      //cout << "Loading file: " << filenames[file_counter] << endl;

      VectorXd vec{FEA_SIZE};
      for(uint i = 0; i < FEA_SIZE; ++i){
	ifdata >> vec(i);
      }
 
      uint label;
      iflabel >> label;

      labels.push_back(label);

      if(labelist.find(label) != labelist.end()){
	labelist[label].push_back(data_idx);
      } else {
	labelist[label] = vector<uint>();
	labelist[label].push_back(data_idx);
      }
      
      vec.normalize();
	  
      data.push_back(vec);
	  
      ++data_idx;

      ifdata.close(); iflabel.close();
    }
  cout << "file size: " << labels.size()
       << "  label size " << labelist.size() << endl;

  for(auto &x: labelist){
    cout << x.first << " " << x.second.size() << endl;
  }
  
  for(uint i = 0; i < NUM_CLASS; ++i)
    {
      VectorXd vec = VectorXd::Zero(NUM_CLASS);
      sim.push_back(vec);
    }
}

void feed_sim(const char* type)
{
  // compute similarity inside each class
  auto labelist_iter = labelist.begin();
  
  for(uint i = 0; i < NUM_CLASS; ++i)
    {
      uint class_size = (labelist_iter->second).size();//labelist[i].size();

      VectorXd totsim(class_size * (class_size-1) / 2);

      auto &list = labelist_iter->second;

      uint counter = 0;
      for(uint m = 0; m < class_size-1; ++ m)
	for(uint l = m+1; l < class_size; ++l)
	  {
	    uint id1, id2;
	    id1 = list[m]; id2 = list[l];
	    
	    totsim(counter++) = compute_similarity(data[id1], data[id2], type);

	  }
      
      sim[i](i) = totsim.sum() / (double)totsim.size();

      cout << sim[i](i) << endl;

      ++labelist_iter;
    }

  auto it1 = labelist.begin();
  
  for(uint i = 0; i < NUM_CLASS-1; ++i){

    auto it2 = it1;
    
    for(uint j = i + 1; j < NUM_CLASS; ++j)
      {
	auto &listi = it1->second;
	auto &listj = it2->second;

	uint count = 0;
	double var = 0.0;
	for(auto &x: listi)
	  {
	    
	    for(auto &y: listj)
	      {
	
		var += compute_similarity(data[x], data[y], type);
		++count;
	      }
	  }
	sim[j](i) = sim[i](j) = var / (double)count;

	++it2;
      }

    ++it1;
  }
}

void save_sim(const char* dir)
{
  ofstream of(dir);

  for(uint i = 0; i < sim.size(); ++i)
    {
      of << sim[i].transpose() << endl;
    }

  of.close();
}

double compute_similarity(VectorXd& v1, VectorXd& v2, const char* type)
{

  if(!strcmp(type, "dot"))
    {
      return v1.dot(v2) / (v1.norm() * v2.norm());
    }
  else if(!strcmp(type, "Gaussian"))
    {
      double sigma = 1.0;
      v1 = v1.normalized();
      v2 = v2.normalized();
      return exp(-(v1 - v2).squaredNorm() / (2*sigma));
    }
  else if(!strcmp(type, "exp"))
    {
      double sigma = 4;
      return exp(-(v1 - v2).norm() * sigma);
    }
  else if(!strcmp(type, "RQ"))
    {
      double c = 0.1;
      double norm = (v1-v2).squaredNorm();
      
      return 1.0 / (norm / c + 1.0);
    }
  else if(!strcmp(type,"histogram"))
    {
      auto min = v1.cwiseMin(v2);
      return min.sum();
      
    }
  else if(!strcmp(type, "power"))
    {
      double d = 1.0;
      double norm = (v1-v2).norm();
      return pow(norm, d);
    }
  else if(!strcmp(type, "InvMQ")) // inverse multiquadratic
    {
      double csquare = 1.0;
      double norm = (v1-v2).squaredNorm();
      return 1.0 / sqrt(norm + csquare);
    }
  else if(!strcmp(type,"polynomial"))
    {
      double c = -4.5;
      double d = 4;
      return pow(v1.dot(v2) + c, d);
    }
  else if(!strcmp(type,"circular"))
    {
      auto diff = (v1-v2).cwiseAbs();
      double sum = 0.0;
      for(uint i = 0; i < diff.size(); ++i)
	sum += exp(-diff(i)/2);
      return sum;
    }
  else if(!strcmp(type, "CityBlock"))
    {
      return (v1 - v2).cwiseAbs().sum();
    }
  else if(!strcmp(type, "Max"))
    {
      return (v1-v2).cwiseAbs().maxCoeff();
    }
  else if(!strcmp(type, "new"))
    {
      double diff = (v1-v2).squaredNorm() + 1.0;
      // double sigma = 4.5;
      // return exp(-sigma * diff);
      //cout << diff << endl;
      double c = 0.05;
      return 1.0 / (diff / pow(c, diff) + 1.0);
    }
  
  
}
