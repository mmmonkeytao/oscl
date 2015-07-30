#include "io.h"
#include "proc.h"
#include "hmp.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <fstream>
#include <chrono>

using namespace oscl;
using namespace oscl::omp;
using namespace cv;
using namespace Eigen;
using namespace std;
using namespace engine;
using namespace std::chrono;

int main(){

      HMP hmp("hmp.config", "rgbd","second+first");
      char rgbdir[] = "apple_1_1_1_crop.png";
      char depthdir[] = "apple_1_1_1_depthcrop.png";

      VectorXd fea;
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      hmp.computeHMP(rgbdir,depthdir, fea);
      high_resolution_clock::time_point t2 = high_resolution_clock::now();

      cout << "Time spent for HMP feature(s): "
	       << duration_cast<milliseconds>(t2-t1).count()/1000.0<<endl;

			
      // Mat X;
      // X = cvCreateMat(100,100,CV_8UC3);
      // randu(X, Scalar::all(0), Scalar::all(0));
      // cout << X << endl << endl;

      // cout << X-1 << endl;
      // imshow("window",X);
      // waitKey(0);
      //ofstream ofile("patchMat.dat");
      //ofile << fea << endl;
      //ofile.close();

      
      return 0;
}
