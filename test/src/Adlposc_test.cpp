#include "LPOSC.h"

#include <iostream>
#include <sstream>
#include <chrono>
#include <time.h>

using namespace std;
using namespace oscl;
using namespace std::chrono;

uint NUM_FILES;
uint FEA_SIZE;
const uint NUM_SAVE_PER_ITER = 500;

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

  LPOSC lposc(FEA_SIZE, "exp", sigma, 0.125);

  uint fea_counter = 0;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(uint i = 0; i < NUM_FILES; ++i){
    stringstream ss;
    ss << i+1;
    string path("_");
    path = prefix + path + ss.str() + suffix;
    string labelpath = prefix.substr(0, prefix.find_last_of("/")+1) + labelprefix + ss.str() + ".dat";
    cout << path << endl;
    cout << labelpath << endl;
    cout << "current file:" << i+1 << endl;

    ifstream ilabel(labelpath.c_str()), idata(path.c_str());
    //ifstream ilabel("2Dclusterslabels.dat"), idata("2DClusters.dat");
    uint fea_num, fea_size;
    idata >> fea_num >> fea_size;
    
    for(uint j = 0; j < fea_num; ++j){

      VectorXd v{fea_size};

      double label, var;
      ilabel >> label; 

      for(uint k = 0; k < fea_size; ++k)
	idata >> v(k);

      //if(i == 0 && j < 100)
      //lposc.preInitData(v, static_cast<int>(label));
      // else
      //lposc.insert1(v, static_cast<int>(label));
      v.normalized();
      lposc.insert2(v, static_cast<int>(label), j, 0.99);
      
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

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  
  cout << duration_cast<seconds>(t2-t1).count() << endl;
  cout << "Insertion completes.\n";
 
  //lposc.save_eval_files("./rgbd2");
  lposc.exportClusterInfo("clusterInfo.dat",0);
  lposc.exportClustDot("Clust.dot");
  lposc.V_measure(1, true);

  return 0;
}
