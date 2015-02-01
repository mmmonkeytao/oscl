#include "OnlineStarClustering.h"
#include <fstream>
#include <Eigen/Dense>
#include <iomanip>

using namespace std;
using namespace Eigen;
using namespace oscl;

const uint NUM_FILE = 1;

int main(int argc, char** argv)
{

  if(argc != 4 && argc != 1){
    cerr << "Usage: <exec> <datadir> <labeldir> <sigma>\n";
    return -1;
  }

  if(argc == 4){

    ifstream iflabel(argv[2]), ifdata(argv[1]);
    uint fea_num, fea_size;

    ifdata >> fea_num >> fea_size;

    vector<VectorXd> data;
    vector<double> label(fea_num);

    for(uint i = 0; i < fea_num; ++i){

      VectorXd v{fea_size};
      iflabel >> label[i];

      for(uint j = 0; j < fea_size; ++j)
	ifdata >> v(j);

      data.push_back(v);
    }
 
    OnlineStarClustering osc(2, "Gaussian", atof(argv[3]));

    for(uint i = 0; i < fea_num; ++i){
      osc.insert(data[i], label[i]);
    }

    osc.V_measure(1, true);

  } else {

    string path_depth21("rgbd2_");
    string labels("label_");

    vector<VectorXd> data;
    vector<double> label;
    uint fea_num, fea_size;

    for(uint i = 0; i < NUM_FILE; ++i){

      stringstream ss;
      ss << i+1;

      string path1 = path_depth21 + ss.str() + ".dat";
      string path2 = labels + ss.str() + ".dat";
      
      ifstream iflabel(path2.c_str()), ifdata(path1.c_str());
      
      ifdata >> fea_num >> fea_size;

      for(uint j = 0; j < fea_num; ++j){

    	double l;
    	VectorXd v{fea_size};
    	iflabel >> l;
    	label.push_back(l);

    	for(uint k = 0; k < fea_size; ++k)
    	  ifdata >> v(k);

    	data.push_back(v);

      }

      iflabel.close(); ifdata.close();
    }

    cout << "loaded data\n";

    double sigma = 0.4;

    while(sigma < 0.95 + 10e-3){      
      
      OnlineStarClustering osc(fea_size, "Gaussian", sigma);

      for(uint i = 0; i < data.size(); ++i){
      	osc.insert(data[i], label[i]);
      }

      osc.V_measure(1, true);

      sigma += 0.05;

      // if(sigma == 0.70){
      // 	ofstream out("simMat.dat");
      // 	out << osc.getSimilarityMat() << endl;
      // 	out.close();
      // }
    }
    
  }

  
    
}
