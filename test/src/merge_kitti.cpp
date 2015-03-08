#include <iostream>
#include <fstream>
#include <sstream>
#include "../../engine/include/proc.h"

using namespace std;

int main(int argc, char**argv){

  if(argc != 4){
    cerr << "Usage: <./Exe> <File_Prefix> <Label_Prefix> <Number of Files>\n";
    return -1;
  }

  const uint num_file = atoi(argv[3]);
  const string file_prefix(argv[1]);
  const string label_prefix(argv[2]);

  ofstream ofileData("rgbd2_data.dat"), ofileLabel("rgbd2_label.dat");
  
  for(uint i = 0; i < num_file; ++i){

    stringstream ss;
    ss << i + 1;

    string ifilepath = file_prefix + ss.str() + ".dat";
    string ilabelpath = label_prefix + "label_" + ss.str() + ".dat";
    
    ifstream ifileData(ifilepath.c_str());
    ifstream ifileLabel(ilabelpath.c_str());

    double bufd;
    uint bufi;

    uint num_fea, fea_size;
    ifileData >> num_fea >> fea_size;

    for(uint j = 0; j < num_fea; ++j){

      for(uint k = 0; k < fea_size; ++k){
	ifileData >> bufd;
	ofileData << bufd << " ";
      }

      ofileData << endl;

      ifileLabel >> bufi;
      ofileLabel << bufi << endl;
    }

    ifileData.close();
    ifileLabel.close();

    cout << "Complet: " << i+1 << endl;
  }

  ofileData.close();
  ofileLabel.close();
  
  return 0;
}
