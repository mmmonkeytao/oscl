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
  
  Galaxy galaxy(FEA_SIZE, argv[1]);

  /*time measurement*/
  long int ttot = 0;
  high_resolution_clock::time_point tt1, tt2;
  
  for(uint i = 0; i < NUM_FEAS; ++i){
    /* read input file */
    stringstream ss;
    ss << i;

    /* load label*/
    string label_path = labelprefix + "label_" + ss.str() +".dat";
    ifstream ilabel(label_path.c_str());

    uint label;  ilabel >> label;  ilabel.close();

    /* load vector*/
    VectorXd vec{FEA_SIZE};
    string data_path = prefix + "_" + ss.str() + ".dat";
    ifstream idata(data_path.c_str());
    for(uint k = 0; k < FEA_SIZE; ++k)
      idata >> vec(k);

    /* insert data */
    // print label and data path
    //cout << label_path << "\n" << data_path << endl;
    tt1 = high_resolution_clock::now();
    galaxy.guided_osc_insert(vec, i, label, i, 0.9);
    tt2 = high_resolution_clock::now();

    ttot += duration_cast<milliseconds>(tt2-tt1).count();
    // release vec
    vec = VectorXd();
    
    if( (i+1)%NUM_SAVE_PER_ITER == 0 || i == (NUM_FEAS-1) ){

      cout <<"Inserted  " << (i+1) << endl; 
      cout << "Time spent(s): " << (long double)ttot / 1000.0 << endl;
      galaxy.Hquery_accuracy();
      galaxy.V_measure(1, true);

      auto vcnum = galaxy.getCenterCheckNum();
      //auto vsbnum = galaxy.getStarBrokenNum();
      auto cschangednum = galaxy.getCSChangedNum();

      cout << "Star Center Broken: " << cschangednum[i] << endl;
      cout << "Center evaluated: " << vcnum[i] << endl;
      //cout << "Center star broken: " << vsbnum[i] << endl;

      ttot = 0;
      
      cout << "\n";
    }
  }
  
  cout << "Insertion completes.\n";
 
  galaxy.exportClusterInfo("clusterInfo.dat",0);
  galaxy.exportClustDot("Clust.dot");
  
  return 0;
}
