#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <map>
#include <fstream>

#include "proc.h"
#include "hmp.h"

using namespace Eigen;
using namespace oscl;
using namespace oscl::engine;
using namespace std;
using namespace std::chrono;

//#define NUM_SAMPLE_OBJ 10
#define DATA_DIR "../../data/rgbdeval"
#define SAVE_DIR "."
#define FEA_SIZE (14*1200 + 14*500 + 14*2400 + 14*1000)*2 


int main()
{

  map<string, uint> _label;
  uint label_counter = 0;

  vector<string> subdir_names;
  string data_dir(DATA_DIR);
  string save_dir(SAVE_DIR);
  
  // get all first level sub-dir
  sub_dir(DATA_DIR, subdir_names);

  // // create all labels
  for(uint i = 0; i < subdir_names.size(); ++i)
    _label[subdir_names[i]] = label_counter++;

  // save all the labels to file
  cout << "Saving labels to file " << save_dir << "/label.dat" << endl; 
  ofstream ofile2((save_dir + "/label.dat").c_str());
  for(auto it = _label.begin(); it != _label.end(); ++it)
    ofile2 << it->first << " " << it->second << endl;
  ofile2.close();

  // create HMP objet and load dictionaries
  HMP hmp("hmp.config", "rgbd", "second+first");

  //
  uint file_counter = 0;
  double tot_time = 0.0;

  for(uint i = 10; i < subdir_names.size(); ++i){

    // feature mat and lable mat to be written into files
    vector<VectorXd> feaMat;
    VectorXd rgbdfea21{FEA_SIZE};
    
    // current_label is also subdir_names[i]
    string subdir = data_dir + "/" + subdir_names[i];

    vector<string> subsub_names;
    sub_dir(subdir.c_str(), subsub_names);

    // for each subsub_dir, read all image and depth files
    for(uint j = 0; j < subsub_names.size(); ++j){

      string subsubdir = subdir + "/" + subsub_names[j];

      // get all cropped images
      vector<string> img_paths;
      sub_dir_files(subsubdir.c_str(), "_crop.png", img_paths);

      // for each rgb and depth image, perform hmp features
      for(uint k = 0; k < img_paths.size(); ++k){

	// get depth image and features
  	string deppath = img_paths[k].substr(0, img_paths[k].size()-9);
  	deppath += "_depthcrop.png";

  	// get rgb hmp features
  	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	hmp.computeHMP(img_paths[k].c_str(), deppath.c_str(), rgbdfea21);
  	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	
  	cout << "Time spent for HMP feature of depth and rgb: "
	     << duration_cast<milliseconds>(t2-t1).count()<<endl;
	
  	// push
  	feaMat.push_back(rgbdfea21);
  	tot_time += duration_cast<milliseconds>(t2-t1).count();
  	++file_counter;
      }            
    }
    
    // save features into file
    // format rows, cols \n label pos value.....
    cout << "Start saving sparse features for " << subdir_names[i] << endl;
    string str21 = save_dir + "/" + subdir_names[i] + "_rgbd21.dat";
    ofstream ofile1(str21.c_str());
    
    ofile1 << feaMat.size() <<"  "<< FEA_SIZE << endl;
    ofile1 << i << endl;
    
    for(uint line = 0; line < feaMat.size(); ++line){
      for(uint rows = 0; rows < feaMat[line].size(); ++rows)
  	if(feaMat[line](rows) != 0.0){
  	  ofile1 << rows <<" "<< feaMat[line](rows) << " ";
  	}
      
      ofile1 << endl;
    }      
    ofile1.close();
    
  }

  cout << "Total file processed: " << file_counter << endl
       << "Average processing time: " << tot_time/(double)file_counter/1000 << endl;

  return 0;
}
