#include "OnlineStarClustering.h"

void oscl::OnlineStarClustering::loadAndAddData(char const *filename)
{
  std::ifstream ifile(filename);

  Eigen::VectorXd vec;
  ///////////////////////////
  double label;  int dataID = 0;
  ///////////////////////////

  // data dimension
  int dim  = 0, start_idx = _data.size();
  if(_data.size() != 0)
    dim = _data.back().size();

  do {
    vec = readVector(ifile);
    if(dim == 0)
      if(vec.size() == 0)
	throw "Could not read data file";
      else
	dim = vec.size();

    else if (dim != vec.size() && vec.size() != 0)
      throw "Invalid length of data vector";

    if(vec.size()){
      //vec /= vec.norm();
      ////////////////////////////////////////
      Eigen::VectorXd vector(vec.size()-1);
      label = vec(0);
      for(int i = 0; i < vec.size() - 1; i++){
        vector(i) = vec(i+1);
      }

      ////////////////////////////////////////
      _data.push_back(vector);
      //////////////////////////
      _labels[dataID] = label;
      dataID++;
      //////////////////////////
    }
  } while (vec.size() != 0);

  //updateSimilarityMatrix(start_idx);
  insertData(start_idx);
  //cout<<"after insert data: "<< _graph.size()<<endl;
  /*
  for(map<uint,Vertex>::const_iterator it1 = _graph.begin(); 
      it1 != _graph.end(); ++it1){

      list<uint> adj_list = it1->second.getAdjVerticesList();

      cout << "Point " << it1->first << " Adj list:"<<endl;
      for(list<uint>::const_iterator it2 = adj_list.begin(); 
	  it2 != adj_list.end(); ++it2)
	cout << *it2 << " ";

      cout << endl;
  }
  */
  ifile.close();
}

void oscl::OnlineStarClustering::loadAndAddSparseData(char const *filename)
{
  std::ifstream ifile(filename);

  ///////////////////////////
  double label;  int dataID = 0;
  ///////////////////////////

  // data dimension
  int dim  = 0, start_idx = _data.size();
  if(_data.size() != 0)
    dim = _data.back().size();

  // read number and size of features 
  uint lines, size;
  VectorXd firstline;
  firstline = readVector(ifile);
  lines = static_cast<uint>(firstline(0));
  size = static_cast<uint>(firstline(1));

  Eigen::VectorXd spVec;
  do {
    Eigen::VectorXd vec = Eigen::MatrixXd::Zero(size,1);
    spVec = readVector(ifile);

    if(spVec.rows()){

      label = spVec(0);
      
      for(uint i = 1; i < spVec.size(); i+=2){
	uint idx = static_cast<uint>(spVec(i));
	vec(idx) = spVec(i+1);	
      }
      
      ////////////////////////////////////////
      _data.push_back(vec);
      //////////////////////////
      _labels[dataID] = label;
      dataID++;
      //////////////////////////
    }
  } while (spVec.rows() != 0);

  //updateSimilarityMatrix(start_idx);
  insertData(start_idx);
  //insertSparseData();
  //cout<<"after insert data: "<< _graph.size()<<endl;
  /*
  for(map<uint,Vertex>::const_iterator it1 = _graph.begin(); 
      it1 != _graph.end(); ++it1){

      list<uint> adj_list = it1->second.getAdjVerticesList();

      cout << "Point " << it1->first << " Adj list:"<<endl;
      for(list<uint>::const_iterator it2 = adj_list.begin(); 
	  it2 != adj_list.end(); ++it2)
	cout << *it2 << " ";

      cout << endl;
  }
  */
  ifile.close();
}

Eigen::VectorXd oscl::OnlineStarClustering::readVector(std::ifstream &ifile) const
{
  std::string line;
  getline(ifile, line);

  size_t pos = 0;
  std::vector<double> vals;

  do {
    size_t next_pos    = line.find_last_of(' ',line.find_first_of(' ', pos+1) + 1);
    std::string substr = line.substr(pos, next_pos - pos);
    double val;

    if(substr == "inf" || substr == "INF")
      vals.push_back(HUGE_VAL);
    else {
      if(sscanf(substr.c_str(), "%lf", &val) == 1){
	       vals.push_back(val);
      }
    }
    
    pos = next_pos;
  } while(pos && pos < line.size());
  
  Eigen::VectorXd vec(vals.size());
  for(uint i=0; i<vals.size(); ++i)
    vec(i) = vals[i];
  
  return vec;
}
