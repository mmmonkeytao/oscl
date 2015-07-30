#include "Galaxy.h"
#include "Planet.h"

#include <iostream>
#include <sstream>
#include <chrono>
#include <time.h>
#include <Eigen/Dense>

using namespace std;
using namespace oscl;
using namespace std::chrono;
using namespace Eigen;


uint global_counter = 0;

uint NUM_FEAS;
uint FEA_SIZE;
const uint NUM_SAVE_PER_ITER = 500;

int main(int argc, char** argv){

  if(argc != 4){
    cerr << "Usage: <exec> <prefix_path(e.g. /rgbd2)> <NUM_FEAS> <FEA_SIZE>\n";
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

  global_counter = 0;

  // vector used to compute H queries
  Eigen::VectorXi hquery = Eigen::VectorXi::Zero(NUM_FEAS);

  //galaxy.hquery = Eigen::VectorXi::Zero(NUM_FEAS);
  
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

    //if(i==1000)
    //global_counter = 0;
    /* insert data */
    // print label and data path
    //cout << label_path << "\n" << data_path << endl;
    tt1 = high_resolution_clock::now();
    //galaxy.loaded_insert_noSim(vec, i, label, i, 0.9);
    //bool flag = galaxy.loaded_insert_noSim(vec, i, label, global_counter, 0.9);
    //galaxy.normal_osc_insert(vec, i, label, i, 0.9);
    bool flag = galaxy.nearest_neighbors_insert(vec, i, label, i, 0.9);
    tt2 = high_resolution_clock::now();
    //++global_counter;
    hquery(i) = flag;
    
    // if(false)
    //   global_counter = 1;
    // else
    //   ++global_counter;
    
    ttot += duration_cast<milliseconds>(tt2-tt1).count();
    // release vec
    vec = VectorXd();
    
    if( (i+1)%NUM_SAVE_PER_ITER == 0 || i == (NUM_FEAS-1) ){

      cout <<"Inserted  " << (i+1) << endl; 
      cout << "Time spent(s): " << (long double)ttot / 1000.0 << endl;
      galaxy.Hquery_accuracy();
      galaxy.V_measure(1, true);

      //auto vcnum = galaxy.getCenterCheckNum();
      //auto vsbnum = galaxy.getStarBrokenNum();

      //cout << "Center evaluated: " << vcnum[i] << endl;
      //cout << "Center star broken: " << vsbnum[i] << endl;
      
      //ofstream ofile("hquery.dat");
      //ofile << hquery.transpose();
      //ofile.close();

      ttot = 0;
      
      cout << "\n";
    }
  }
  
  cout << "Insertion completes.\n";

  galaxy.propagate_save_label();
  
  for(uint i = 1; i < NUM_FEAS; ++i)
    hquery(i) = hquery(i)+hquery(i-1);
  
  ofstream ofile("hquery.dat");
  ofile << hquery.transpose() << endl;
  ofile.close();

  galaxy.saveCenterList();
  galaxy.exportClusterInfo("clusterInfo.dat",0);
  //galaxy.exportClustDot("Clust.dot");

  //galaxy.saveRequireLabel("RequireLabels.dat");
  //galaxy.saveCenterCheckNum("CenterCheckNum.dat");
  //galaxy.saveStarBrokenNum("StarCenterBrokenNum.dat");
  
  return 0;
}
