#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <map>
#include <fstream>
#include <algorithm> 

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

  map<uint, string> _label;
  uint label_counter = 0;

  vector<string> subdir_names;
  string data_dir(DATA_DIR);
  string save_dir(SAVE_DIR);
  
  // get all first level sub-dir
  sub_dir(DATA_DIR, subdir_names);

  // // create all labels
  for(uint i = 0; i < subdir_names.size(); ++i)
    _label[label_counter++] = subdir_names[i];

  vector<uint> totlabel;
  vector<string> imgnm;
  vector<string> totdir;
  //
  uint file_counter = 0;
  double tot_time = 0.0;
  for(uint i = 0; i < subdir_names.size(); ++i){
   
    // current_label is also subdir_names[i]
    string subdir = data_dir + "/" + subdir_names[i];

    vector<string> subsub_names;
    sub_dir(subdir.c_str(), subsub_names);

    //cout << _label[i] << endl;
    // for each subsub_dir, read all image and depth files
    for(uint j = 0; j < subsub_names.size(); ++j){

      string subsubdir = subdir + "/" + subsub_names[j];

      // get all cropped images
      vector<string> img_paths;
      sub_dir_files(subsubdir.c_str(), "_crop.png", img_paths);

      for(uint k = 0; k < img_paths.size(); ++k){
	totdir.push_back(img_paths[k]);
	totlabel.push_back(i);
	++file_counter;
      } 
    }       
  }

  vector<uint> rdmIdx;
  for(uint i = 0; i < file_counter; ++i)
    rdmIdx.push_back(i);

  random_shuffle(rdmIdx.begin(), rdmIdx.end());
  random_shuffle(rdmIdx.begin(), rdmIdx.end());

  // create HMP objet and load dictionaries
  // store features for every 1000
  HMP hmp("../hmp.config", "rgbd", "second+first");

  vector<VectorXd> feas;
  
  for(uint i = 0; i < rdmIdx.size(); ++i){

    string rgbpath = totdir[ rdmIdx[i] ];

    // get depth image and features
    string deppath = rgbpath.substr(0, rgbpath.size()-9);
    deppath += "_depthcrop.png";

    VectorXd rgbdfea21;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    hmp.computeHMP(rgbpath.c_str(), deppath.c_str(), rgbdfea21);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    cout << "Time spent for HMP feature of depth and rgb: "
    	 << duration_cast<milliseconds>(t2-t1).count()<<endl
	 <<"Current Iteration: " << i << endl;

    feas.push_back(rgbdfea21);
    
    if( (i+1)%1000 == 0 || i == rdmIdx.size()-1 ){
      // save features into file
      cout << "Start saving features for it: " << i << endl;
      uint startid = i + 1 - feas.size();
      
      string rgbdir21("rgbd21_");
      string rgbdir2("rgbd2_");
      string rgbdir1("rgbd1_");
      string depthdir21("depth21_");

      string labeldir("label_");
      string picnmdir("picnm_");

      stringstream id;
      id << ceil ((double)(i+1)/1000.0);

      rgbdir21 += id.str() + ".dat";
      rgbdir2 += id.str() + ".dat";
      rgbdir1 += id.str() + ".dat";
      depthdir21 += id.str() + ".dat";
      labeldir += id.str() + ".dat";
      picnmdir += id.str() + ".dat";
   
      // open file to store
      ofstream ofrgbd21(rgbdir21.c_str());
      ofstream ofrgbd2(rgbdir2.c_str());
      ofstream ofrgbd1(rgbdir1.c_str());
      ofstream ofd21(depthdir21.c_str());
      ofstream ofl(labeldir.c_str());
      ofstream ofp(picnmdir.c_str());

      // save num fea and fea size
      ofrgbd21 << feas.size() <<" " << FEA_SIZE << endl;
      ofrgbd2 << feas.size() << " " << 14*(1000+500)*2 << endl;
      ofrgbd1 << feas.size() << " " << 14*(1200+2400)*2 << endl;
      ofd21 << feas.size() << " " << 14*(1200+500+2400+1000) << endl;

      uint current_id = startid;
      for(uint c = 0; c < feas.size(); ++c){

	if( (c+1)%100 == 0){
	  cout << "saving complete iteration: " << c << endl;
	}
	
	string p = totdir[ rdmIdx[startid+c] ];
	string picname = p.substr(p.find_last_of("/")+1, string::npos);

	VectorXd rgbdfea2{14*(1000+500)*2};
	VectorXd rgbdfea1{14*(1200+2400)*2};
	VectorXd depth21{14*(1200+500+2400+1000)};

	depth21 = feas[c].head(depth21.size()) * sqrt(8);
	depth21.normalize();

	rgbdfea2.head(7000) = feas[c].head(7000);
	rgbdfea2.segment(7000, 14000) = feas[c].segment(23800,14000);
	rgbdfea2.segment(7000+14000,7000) = feas[c].segment(71400,7000);
	rgbdfea2.tail(14000) = feas[c].segment(95200,14000);
	rgbdfea2 *= sqrt(8);
	rgbdfea2.normalize();

	rgbdfea1.head(14*1200) = feas[c].segment(7000,14*1200);
	rgbdfea1.segment(14*1200,14*2400) = feas[c].segment(37800, 14*2400);
	rgbdfea1.segment(14*3600, 14*1200) = feas[c].segment(78400, 14*1200);
	rgbdfea1.tail(14*2400) = feas[c].tail(14*2400);
	rgbdfea1 *= sqrt(8);
	rgbdfea1.normalize();
	
	ofrgbd21 << feas[c].transpose() << endl;
	ofrgbd2 << rgbdfea2.transpose() << endl;
	ofrgbd1 << rgbdfea1.transpose() << endl;
	ofd21 << depth21.transpose() << endl;

	ofl << totlabel[ rdmIdx[startid+c] ] << endl;
	ofp << picname << endl;
	
      }

      ofrgbd21.close();
      ofrgbd2.close();
      ofrgbd1.close();
      ofd21.close();
      ofl.close();
      ofp.close();      

      feas = vector<VectorXd>();
    }
    
  }
  
  // cout << "Total file processed: " << file_counter << endl
  //      << "Average processing time: " << tot_time/(double)file_counter/1000 << endl;

  return 0;
}
