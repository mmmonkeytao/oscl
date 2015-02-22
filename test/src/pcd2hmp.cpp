#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <map>
#include <fstream>
#include <algorithm> 
#include <opencv2/opencv.hpp>
#include <sstream>

#include "proc.h"
#include "hmp.h"

using namespace Eigen;
using namespace oscl;
using namespace oscl::engine;
using namespace std;
using namespace std::chrono;
using namespace cv;

//#define NUM_SAMPLE_OBJ 10
#define DATA_DIR "../convert2DDepNorm/"
#define SAVE_DIR "."
#define FEA_SIZE (14*1200 + 14*500 + 14*2400 + 14*1000) 


int main()
{

  map<uint, string> _label;
  uint label_counter = 0;

  vector<string> subdir_names;
  string data_dir(DATA_DIR);
  string save_dir(SAVE_DIR);

  ofstream offea("hmpfea.dat"), oflabel("label.dat");

  offea << 1108 << " " << FEA_SIZE << endl;
  // create HMP objet and load dictionaries
  HMP hmp("hmp.config", "depth", "second+first");

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  
  for(uint i = 0; i < 1108; ++i){

    stringstream ss; ss << i;
    string path = DATA_DIR + ss.str() + ".dat";

    ifstream ifile(path.c_str());

    int label, width, height;

    ifile >> height >> width >> label;
    cv::Mat depth,  normal;
    depth = cvCreateMat(height, width, CV_64FC1);
    normal = cvCreateMat(height, width, CV_64FC3);
    
    for(uint col = 0; col < width; ++col)
      for(uint row = 0; row < height; ++row)
    	{
    	  double var1, var2, var3, var4;
    	  cv::Vec<double,3> v;

    	  ifile >> var1 >> var2 >> var3 >> var4;

    	  depth.at<cv::Vec<double,1> >(row, col) = var1;
    	  normal.at<cv::Vec<double,3> >(row, col) = cv::Vec<double,3>(var2, var3, var4); 
    	}

    Eigen::VectorXd hmpfea;
    hmp.computeHMPfromPCD(depth, normal, hmpfea);

    offea << hmpfea.transpose() << endl;
    oflabel << label << endl;

    cout << "Complete: " << i << endl;
  }

  offea.close();
  oflabel.close();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  cout << "Total computation time: " << duration_cast<seconds>(t2-t1).count() << " file no.: " << 1108 << endl;
  return 0;
}
