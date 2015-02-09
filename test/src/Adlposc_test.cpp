#include "LPOSC.h"

#include <iostream>
#include <sstream>
#include <time.h>

using namespace std;
using namespace oscl;

uint NUM_FILES;
uint FEA_SIZE;
const uint NUM_SAVE_PER_ITER = 100;

int main(int argc, char** argv){

  if(argc != 5){
    cerr << "Usage: <exec> <prefix_path> <sigma> <NUM_FILES> <FEA_SIZE>\n";
    return -1;
  }
  
  string prefix(argv[1]);
  string suffix(".dat");
  string labelprefix("label_");

  double sigma = atof(argv[2]);
  NUM_FILES = atoi(argv[3]);
  FEA_SIZE = atoi(argv[4]);
  
  cout <<"Sigma is:"<< sigma << endl;

  LPOSC lposc(FEA_SIZE, "Gaussian", sigma);

  uint fea_counter = 0;

  clock_t t  = clock();
  
  for(uint i = 0; i < NUM_FILES; ++i){
    stringstream ss;
    ss << i+1;
    string path("_");
    path = prefix + path + ss.str() + suffix;
    labelprefix = prefix.substr(0, prefix.find_last_of("/")+1) + labelprefix + ss.str() + ".dat";

    cout << "current file:" << i+1 << endl;

    ifstream ilabel(labelprefix.c_str()), idata(path.c_str());
    //ifstream ilabel("2Dclusterslabels.dat"), idata("2DClusters.dat");
    uint fea_num, fea_size;
    idata >> fea_num >> fea_size;
    
    for(uint j = 0; j < fea_num; ++j){

      VectorXd v{fea_size};
      //SpVec v(fea_size);
      double label, var;
      ilabel >> label;

      for(uint k = 0; k < fea_size; ++k){
	idata >> v(k);
	//idata >> var;
	//if(var != 0.0)
	//v.insert(k,0) = var;
      }

      if( j < 100 && i == 0 )
      	lposc.preInitData(v, static_cast<int>(label));
      else
      	lposc.insert1(v, static_cast<int>(label));

      //lposc.insert(v, static_cast<int>(label));
      
      if( (fea_counter+1)%NUM_SAVE_PER_ITER == 0 ||
	  (i == (NUM_FILES-1) && j == fea_num-1) ){

#ifdef OPTIMIZE
	clock_t t = clock();
#endif
	//lposc.calc_Vmeasure();	
	//lposc.center_label_propagation("threshold");
	//lposc.center_label_propagation("current_edge");
	//lposc.center_label_propagation("all_edge");
#ifdef OPTIMIZE
	t = clock()-t;
#endif
	cout << "Inserted " << fea_counter << "\n";
#ifdef OPTIMIZE
	cout << "Time spent for label_propagation: "<< (float)t/CLOCKS_PER_SEC << endl << endl;
#endif	
      }
 
      ++fea_counter;
    }
    ilabel.close();
    idata.close();
  }

  t = clock() - t;
  cout << (float)t/CLOCKS_PER_SEC << endl;
  cout << "Insertion completes.\n";

  //lposc.save_eval_files("./rgbd2");
  lposc.exportClusterInfo("clusterInfo.dat",0);
  lposc.exportDot("Clust.dot", false);
  lposc.V_measure(1, true);

  return 0;
}
