#include "Galaxy.h"

#include <iostream>
#include <sstream>
#include <chrono>
#include <time.h>

using namespace std;
using namespace oscl;
using namespace std::chrono;

uint NUM_FEAS;
uint FEA_SIZE;
const uint NUM_SAVE_PER_ITER = 200;

int main(int argc, char** argv){

  if(argc != 4){
    cerr << "Usage: <exec> <prefix_path> <NUM_FEAS> <FEA_SIZE>\n";
    return -1;
  }

  string prefix(argv[1]);
  string labelprefix = prefix.substr(0, prefix.find_last_of('/')+1);
  
  NUM_FEAS = atoi(argv[2]);
  FEA_SIZE = atoi(argv[3]);
  
  Galaxy galaxy(FEA_SIZE, "CityBlock", 0,  argv[1]);

  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
  for(uint i = 0; i < NUM_FEAS; ++i){
    /* read input file */
    stringstream ss;
    ss << i;

    string label_path = labelprefix + "label_" + ss.str() +".dat";
    ifstream ilabel(label_path.c_str());

    uint label;
    ilabel >> label; 
    ilabel.close();
    
    galaxy.unloaded_insert(i, label, i, 0.99);
            
    if( (i+1)%NUM_SAVE_PER_ITER == 0 || i == (NUM_FEAS-1) ){
	
      cout <<"Inserted  " << (i+1) << endl; 
	
	galaxy.V_measure(1, true);

	cout << "\n";
    }
  }
  
  cout << "Insertion completes.\n";
 
  galaxy.exportClusterInfo("clusterInfo.dat",0);
  galaxy.exportClustDot("Clust.dot");
  
  return 0;
}
