#include "Galaxy.h"

using namespace std;
using namespace oscl;

/*
 * constructor and destructor
 *
 */
Galaxy::Galaxy(uint feaSize, const char* file):
_feaSize(feaSize), _fea_path_prefix(file)
{
  _numClusters = 0;
  _h = 0.0;
  _c = 0.0;
  _datasize = 0;
  _path_prefix = _fea_path_prefix.substr(0, _fea_path_prefix.find_last_of('/') + 1);  
}


Galaxy::Galaxy(uint feaSize, string similaritype): _feaSize(feaSize), _simtype(similaritype)
{
  _numClusters = 0;
  _h = 0.0;
  _c = 0.0;
  _datasize = 0;
}

Galaxy::Galaxy(uint feaSize, string similaritype, double param): _feaSize(feaSize), _simtype(similaritype), _param(param)
{
  _numClusters = 0;
  _h = 0.0;
  _c = 0.0;
  _datasize = 0;
}

Galaxy::Galaxy(uint feaSize, string similaritype, double param, const char* file): _feaSize(feaSize), _simtype(similaritype), _param(param), _fea_path_prefix(file)
{
  _numClusters = 0;
  _h = 0.0;
  _c = 0.0;
  _datasize = 0;
  _path_prefix = _fea_path_prefix.substr(0, _fea_path_prefix.find_last_of('/') + 1);
}

Galaxy::~Galaxy()
{
  // clear allocated data
  _similarityMatrix = MatrixType(); 
}

/*
 * Access Functions
 *
 */
uint Galaxy::getDataSize(){
  return _datasize; 
}

void Galaxy::incrementDataSize(){
  ++_datasize;
}

const oscl::Galaxy::DataSet& Galaxy::getData(){
  return _data;
}

const std::map<uint, int>& Galaxy::getLabelList(){
  return _labels;
}

const std::map<uint, Planet> Galaxy::getGraph(){
  return _graph;
}

const Eigen::MatrixXd& Galaxy::getSimilarityMat() {
  return _similarityMatrix;
}

const std::vector<uint>& Galaxy::getCenterCheckNum(){

  return center_check_num;
}

const std::vector<uint>& Galaxy::getStarBrokenNum(){

  return star_broken_num;
}

const std::vector<uint>& Galaxy::getCSChangedNum(){

  return _cs_changed;
}
