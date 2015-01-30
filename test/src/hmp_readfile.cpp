#include "io.h"
#include "proc.h"
#include "hmp.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <fstream>
#include <chrono>
#include <ctime>

using namespace oscl;
using namespace oscl::omp;
using namespace cv;
using namespace Eigen;
using namespace std;
using namespace engine;
using namespace std::chrono;


int main(int argc, char **argv){

  if(argc != 4){
    cerr << "Usage:<./exec> <fea_dir> <label_dir> <names_dir>\n";
    return -1;
  }
  
  MatrixXd mat;
  vector<uint> labels;
  vector<string> names;

  ifstream ofdata(argv[1]), oflabels(argv[2]), ofnames(argv[3]);

  uint fea_num, fea_size;
  ofdata >> fea_num >> fea_size;

  mat = MatrixXd(fea_size, fea_num);

  clock_t t = clock();
  for(uint i = 0; i < fea_num; ++i){
    uint buf_label;
    string buf_nm;
    double var;

    oflabels >> buf_label;
    ofnames >> buf_nm;

    labels.push_back(buf_label);
    names.push_back(buf_nm);

    for(uint j = 0; j < fea_size; ++j){
      ofdata >> var;
      mat(j, i) = var;
    } 
  }

  t = clock() - t;
  ofdata.close(); oflabels.close(); ofnames.close();
  
  cout << "Time spent: " <<(float)t/CLOCKS_PER_SEC  << endl;

  MatrixXd similarityMat = MatrixXd::Identity(fea_num, fea_num);

  for(uint i = 0; i < fea_num; ++i)
    for(uint j = i+1; j < fea_num; ++j){
      similarityMat(i, j) = mat.col(i).transpose() * mat.col(j);
      similarityMat(j, i) = similarityMat(i,j);
    }

  ofstream ofile("simMat.dat");
  ofile << similarityMat << endl;
  ofile.close();
  
  return 0;
}
