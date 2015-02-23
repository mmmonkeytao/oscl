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
// #define DATA_DIR "../convert2DDepNorm/"
// #define SAVE_DIR "."
// #define FEA_SIZE (14*1200 + 14*500 + 14*2400 + 14*1000) 

int main(int argc, char**argv)
{

  if(argc != 5)
    {
      cerr << "Usage: <./exec> <infile> <outfile> <fea_size> <hmp_config.dat>.\n";
      return -1;
    }

  const int fea_size = atoi(argv[3]);

  string prefix(argv[1]);
  string pathout(argv[2]);
  
  vector<string> topdir;
  sub_dir(prefix.c_str(), topdir);

  // create HMP objet and load dictionaries
  HMP hmp(argv[4], "depth", "second+first");

  for(uint i = 0; i < topdir.size(); ++i){

    string path = prefix + topdir[i] + "/";
    vector<string> subfiles;
    sub_dir_files(path.c_str(), ".dat", subfiles);
    
    for(uint j = 0; j < subfiles.size(); ++j){

      cout << "\nfile: " << subfiles[j] << endl;

      ifstream ifile(subfiles[j].c_str());
      
      uint width, height, label;
      ifile >> height >> width >> label;
      
      // measure computational time
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
  
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

      // output file
      size_t pos1 = subfiles[j].find_last_of('/');
      size_t pos2 = subfiles[j].find_last_of('.');
      string basename = subfiles[j].substr(pos1+1, pos2-pos1-1);
      string outhmp = pathout + basename + "_hmp.dat";
      string outlabel = pathout + basename + "_label.dat";
      
      ofstream odata(outhmp.c_str()), olabel(outlabel.c_str());
      olabel << label;
      odata << hmpfea << endl;
      //odata.write(reinterpret_cast<char*>(hmpfea.data()), sizeof(double)*fea_size);
      olabel.close();
      odata.close();

      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      cout << "Total computation time: " << duration_cast<milliseconds>(t2-t1).count()  << endl << endl;

    }
  }
  
  return 0;
}
