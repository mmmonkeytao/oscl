#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

uint NUM_CLASS;
uint NUM_FILES;
vector<int> labels;
vector<VectorXd> data;
auto labelist = map<int, vector<uint> >();
string label_prefix;
string data_prefix;
MatrixXd sim;
vector<VectorXd> Vcenter;

void init_data();
void feed_sim(const char*);
double compute_similarity(VectorXd &x1, VectorXd &x2, const char* type);
void save_sim(const char*);

int main(int argc, char** argv){

  if(argc != 7){
    cerr << "Usage: <exec> <label_file_dir> <data_file_dir> <NUM_FILES> <save_dir> <sim_type> <NUM_CLASS>\n";
    return -1;
  }

  label_prefix = string(argv[1]);
  data_prefix = string(argv[2]);
  NUM_FILES = atoi(argv[3]);
  NUM_CLASS = atoi(argv[6]);
  init_data();

  // feeding similarity information
  cout << "feeding sim.\n"; 
  feed_sim(argv[5]);
  cout << "saving file.\n";
  save_sim(argv[4]);
  
  return 0;
}


void init_data()
{
  for(int i = 0; i < NUM_CLASS; ++i)
    {
      labelist[i] = vector<uint>();     
    }
  
  uint data_idx = 0;

  uint fea_num, fea_size;
  for(uint file_counter = 0; file_counter < NUM_FILES; ++file_counter)
    {
      stringstream ss;
      ss << file_counter + 1;
      
      string label_path = label_prefix + "label_" + ss.str() +".dat";
      string data_path = data_prefix + "_" + ss.str() +".dat";
     
      ifstream ifdata(data_path.c_str()), iflabel(label_path.c_str());
      cout << "Loading file: " << file_counter + 1 << endl;
      
      ifdata >> fea_num >> fea_size;

      //fea_num = 3672; fea_size = 94150;
      for(uint fea_num_idx = 0; fea_num_idx < fea_num; ++fea_num_idx)
	{
	  VectorXd vec(fea_size);
	  int l;
	  
	  for(uint fea_size_idx = 0; fea_size_idx < fea_size; ++fea_size_idx){
	    ifdata >> vec(fea_size_idx);
	  }

	  iflabel >> l;
	  
	  labels.push_back(l);
	  labelist[l].push_back(data_idx);
	  vec.normalize();
	  
	  data.push_back(vec);
	  
	  ++data_idx;
	}

      ifdata.close(); iflabel.close();
    }

  for(uint i = 0; i < NUM_CLASS; ++i)
    {
      VectorXd vc = VectorXd::Zero(fea_size);
      Vcenter.push_back(vc);
    }

  sim = MatrixXd(NUM_CLASS, NUM_CLASS);
}

void feed_sim(const char* type)
{

  // computer center similarity of each data set
  for(uint i = 0; i < NUM_CLASS; ++i)
    {

      auto &list = labelist[i];

      for(auto &x: list)
	{
	  Vcenter[i] += data[x];
	}
      Vcenter[i].array() /= (double)list.size();
    }


  // compute similarity inside each class
  for(uint i = 0; i < NUM_CLASS; ++i)
    for(uint j = i; j < NUM_CLASS; ++j)
      {
	auto &list = labelist[j];

	double sum = 0.0;
	for(auto &id: list)
	  {
	    sum += compute_similarity(data[id], Vcenter[i], type);
	  }

	sim(i, j) = sim(j, i) = sum / (double)list.size();
      }
}

void save_sim(const char* dir)
{
  ofstream of(dir);

  of << sim << endl;
  
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
      double sigma = 4.0;
      return exp(-(v1 - v2).squaredNorm() / (2*sigma));
    }
  else if(!strcmp(type, "exp"))
    {
      double sigma = 8;
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
