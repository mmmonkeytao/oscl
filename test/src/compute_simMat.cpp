#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <stxxl/vector>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){

  if(argc != 4){
    cerr << "Usage: <./exe> <file_dir> <number of feas> <fea size>.\n";
    return -1;
  }

  //vector<VectorXd> data;
  
  ifstream ifile(argv[1],ios::in);
  uint nm_feas = atoi(argv[2]);
  uint fea_size = atoi(argv[3]);

  stxxl::VECTOR_GENERATOR<VectorXd>::result data{nm_feas};
  
  for(uint i = 0; i < nm_feas; ++i){

    //VectorXd vec{fea_size};
    data[i] = VectorXd{fea_size};
    
    for(uint j = 0; j < fea_size; ++j){
      ifile >> data[i](j);
    }
    
    //data[i] = std::move(vec);

    if(i%500 == 0)
      cout << i << endl;
  }

  ifile.close();

  cout << "Loading file complete...\n";
  
  //MatrixXd simMat = MatrixXd::Identity(nm_feas, nm_feas);
  typedef stxxl::VECTOR_GENERATOR<VectorXd>::result matrixd;

  matrixd simMat{nm_feas};
  
  for(uint i = 0; i < nm_feas; ++i)
    simMat[i] = VectorXd::Zero(nm_feas);

  
  for(uint i = 0; i < nm_feas; ++i){
    cout << i << endl;
  
    for(uint j = i+1; j < nm_feas; ++j){
      
      double var = (data[i] - data[j]).cwiseAbs().sum();

      simMat[i](j) = simMat[j](i) = var;      
    }
  }

  ofstream ofile("simMat.dat");

  for(uint i = 0; i < nm_feas; ++i)
    ofile << setprecision(10) << simMat[i].transpose() << endl;

  ofile.close();
  
  return 0;
}
