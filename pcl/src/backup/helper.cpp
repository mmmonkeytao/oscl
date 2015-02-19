#include "pcl_func.hpp"

#include <fstream>
#include <stdlib.h>
#include <string>

using namespace std;

void read_parameters(const char *file_name, double &lscale1, double &lscale2, double &lthreshold, double &lsegradius,
                     int &NUM_NEIGHBORS, double &STD_DEVIATION, double &radius, double &distance_threshold)
{
    bool flag = true;
    string str;
    ifstream infile;
    infile.open (file_name);

    getline(infile,str);
    lscale1 = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    lscale2 = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    lthreshold = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    lsegradius = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    NUM_NEIGHBORS = atoi((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    STD_DEVIATION = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    radius = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();

    getline(infile,str);
    distance_threshold = atof((str.substr(str.find_first_of(' ') + 1 , str.length())).c_str());
    str.clear();
    
    infile.close();

    cout << "scale1: " << lscale1 << endl
         << "scale2: " << lscale2 << endl
         << "threshold: " << lthreshold << endl
         << "segradius: " << lsegradius << endl
         << "number of neighbors points for normals: " << NUM_NEIGHBORS << endl
         << "standard deviation for normals: " << STD_DEVIATION << endl
         << "radius for normals: " << radius << endl
	 << "distance threshold: " << distance_threshold << endl;

}

