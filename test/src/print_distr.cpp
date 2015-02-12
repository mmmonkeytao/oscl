#include <iostream>
#include <sstream>
#include <time.h>
#include <vector>
#include <map>
#include <fstream>

using namespace std;

const uint NUM_FILES = 5;


int main(int argc, char** argv){

  if(argc != 2){
    cerr << "Usage: <exec> <label_file_dir>\n";
    return -1;
  }

  string prefix(argv[1]);
  vector<int> labels;
  map<int, uint> labelist;

  for(uint file_counter = 0; file_counter < NUM_FILES; ++file_counter)
    {
      stringstream ss;
      ss << file_counter+1;
      string path = prefix + "label_" + ss.str() + ".dat";
      
      ifstream ifile(path.c_str());

      while(!ifile.eof()){
	int l;
	ifile >> l;
	labels.push_back(l);

	if(labelist.find(l) == labelist.end())
	  labelist[l] = 1;
	else
	  ++labelist[l];
      }
    }
  
  // print distribution
  for(auto &x: labelist)
    {
      //cout << "Label: " << x.first << " distribution: "
	   cout<< (double)x.second / (double)labels.size() << endl;
    }

  // save distribution
  ofstream ofile("distribution.dat");
  for(auto &x: labelist)
    {
      ofile << x.first << " " << x.second << endl;
    }
  ofile.close();
  
  return 0;
}
