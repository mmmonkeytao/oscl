#include "LPOSC.h"

#include <iostream>
#include <sstream>

using namespace std;
using namespace oscl;

const uint NUM_FILES = 42;
const uint FEA_SIZE = 42000;

int main(int argc, char** argv){

  if(argc != 3){
    cerr << "Usage: <exec> <prefix_path> <sigma>\n";
    return -1;
  }

  string prefix(argv[1]);
  string suffix(".dat");
  string labelprefix("label_");

  double sigma = atof(argv[2]);

  cout <<"Sigma is:"<< sigma << endl;

  LPOSC lposc(FEA_SIZE, "Gaussian", sigma);

  uint fea_counter = 0;
  for(uint i = 0; i < NUM_FILES; ++i){
    stringstream ss;
    ss << i+1;
    string path("_");
    path = prefix + path + ss.str() + suffix;
    labelprefix += ss.str() + ".dat";
    cout << path << endl;
    cout << "current file:" << i+1 << endl;
    
    ifstream ilabel(labelprefix.c_str()), idata(path.c_str());
    //ifstream ilabel("2Dclusterslabels.dat"), idata("2DClusters.dat");
    uint fea_num, fea_size;
    idata >> fea_num >> fea_size;   

    for(uint j = 0; j < fea_num; ++j){

      VectorXd v{fea_size};
      double label;
      ilabel >> label;

      for(uint k = 0; k < fea_size; ++k)
	idata >> v(k);

      lposc.insert(v, static_cast<int>(label));

      if( (fea_counter+1)%50 == 0 ||
	  (i == (NUM_FILES-1) && j == fea_num-1) ){
	//lposc.calc_Vmeasure();
	//lposc.center_label_propagation("threshold");
	//lposc.center_label_propagation("current_edge");
	//lposc.center_label_propagation("all_edge");
	cout << "Inserted " << fea_counter << "\n";
      }
 
      ++fea_counter;
    }
    ilabel.close();
    idata.close();
  }
  cout << "Insertion completes.\n";
  lposc.V_measure(1, true);
  lposc.exportClusterInfo("clusterInfo.dat",0);
  //lposc.save_eval_files("./rgbd2");

  return 0;
}
