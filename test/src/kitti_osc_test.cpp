#include "Galaxy.h"
#include "proc.h"

#include <iostream>
#include <sstream>
#include <chrono>
#include <time.h>
#include <algorithm>

using namespace std;
using namespace oscl;
using namespace std::chrono;
using namespace oscl::engine;

uint FEA_SIZE;
const uint NUM_SAVE_PER_ITER = 50;

int main(int argc, char** argv){

  if(argc != 3){
    cerr << "Usage: <exec> <prefix_path> <FEA_SIZE>\n";
    return -1;
  }

  string prefix(argv[1]);
  vector<string> allfiles;
  sub_dir_files(argv[1], "hmp.dat", allfiles);

  // shuffle twice
  random_shuffle ( allfiles.begin(), allfiles.end() );
  random_shuffle ( allfiles.begin(), allfiles.end() );
  
  FEA_SIZE = atoi(argv[2]);
  
  Galaxy galaxy(FEA_SIZE, "CityBlock");

  /*time measurement*/
  long int ttot = 0;
  high_resolution_clock::time_point tt1, tt2;
  
  for(uint i = 0; i < allfiles.size(); ++i){
    /* read input file */
    /* load label*/
    size_t pos = allfiles[i].find_last_of('_');    
    
    string label_path = allfiles[i].substr(0, pos+1) + "label.dat";
    ifstream ilabel(label_path.c_str());

    uint label;  ilabel >> label;  ilabel.close();

    /* load vector*/
    VectorXd vec{FEA_SIZE};

    ifstream idata(allfiles[i].c_str());
    for(uint k = 0; k < FEA_SIZE; ++k)
      idata >> vec(k);
   
    /* insert data */
    // print label and data path
    //cout << label_path << "\n" << data_path << endl;
    tt1 = high_resolution_clock::now();
    //galaxy.loaded_insert_noSim(vec, i, label, i, 0.9);
    //galaxy.normal_osc_insert(vec, i, label, i, 8.0e-3);
    bool flag = galaxy.nearest_neighbors_insert(vec, i, label, i, 0.9);
    tt2 = high_resolution_clock::now();

    ttot += duration_cast<milliseconds>(tt2-tt1).count();
    // release vec
    vec = VectorXd();
    
    if( (i+1)%NUM_SAVE_PER_ITER == 0 || i == (allfiles.size()-1) ){

      cout <<"Inserted  " << (i+1) << endl; 
      cout << "Time spent(s): " << (long double)ttot / 1000.0 << endl;
      galaxy.Hquery_accuracy();
      galaxy.V_measure(1, true);

      //auto vcnum = galaxy.getCenterCheckNum();
      //auto vsbnum = galaxy.getStarBrokenNum();
      //auto cschangednum = galaxy.getCSChangedNum();

      //cout << "Star Center Broken: " << cschangednum[i] << endl;
      //cout << "Center evaluated: " << vcnum[i] << endl;
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
