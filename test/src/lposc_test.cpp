#include "LPOSC.h"

#include <iostream>
#include <sstream>

using namespace std;
using namespace oscl;

const uint NUM_FILES = 1;

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

  LPOSC lposc(71400, "Gaussian", sigma);

  for(uint i = 0; i < NUM_FILES; ++i){
    stringstream ss;
    ss << i+1;
    string path("_");
    path = prefix + path + ss.str() + suffix;
    labelprefix += ss.str() + ".dat";
    cout << path << endl;
    cout << "current file:" << i << endl;
    
    ifstream ilabel(labelprefix.c_str()), idata(path.c_str());
    uint fea_num, fea_size;
    idata >> fea_num >> fea_size;   

    for(uint i = 0; i < fea_num; ++i){

      VectorXd v{fea_size};
      double label;
      ilabel >> label;

      for(uint j = 0; j < fea_size; ++j)
	idata >> v(j);

      lposc.insert(v, static_cast<int>(label));
      lposc.center_label_propagation("threshold");
      lposc.center_label_propagation("current_edge");
      lposc.center_label_propagation("all_edge");
      
      cout << "Inserted " << i << "\n";      
    }
    ilabel.close();
    idata.close();
  }
  cout << "Insertion completes.\n";
  //lposc.V_measure(1, true);
  lposc.save_eval_files(argv[1]);
  
  return 0;
}
