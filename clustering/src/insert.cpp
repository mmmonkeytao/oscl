#include "OnlineStarClustering.h"

void oscl::OnlineStarClustering::insert(VectorXd &vec)
{
  if(static_cast<uint>(vec.size()) != _feaSize)
    throw std::runtime_error("\nFeature inserted has wrong dimention.\n");
  
  // insert into sparse dataset
  _data.push_back(vec);
 
  // insert and update clusters
  insertSparseData();
}

void oscl::OnlineStarClustering::insert(VectorXd vec, int label)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  _data.push_back(vec);
  // label = -1 if no label
  _labels[_data.size()-1] = static_cast<int>(label);

  // adjacent list
  std::list<uint> L;
    
  // update similarity matrix and sigma graph
  _sigmaGraph.conservativeResize(_data.size(), _data.size());
  _similarityMatrix.conservativeResize(_data.size(), _data.size());
  
  uint dataID = _data.size()-1;
  for(uint i = 0; i < _data.size()-1; ++i){

    double sv = computeSimilarity(i, dataID);
    _similarityMatrix(i, dataID) = _similarityMatrix(dataID, i) = sv;

    if ( sv >= _sigma){        
      _sigmaGraph.insert(dataID, i) = 1;
      L.push_back(i);
    }

  }
  
  fastInsert(dataID, L);
  L.clear();
  
}

void oscl::OnlineStarClustering::insertData(uint start_idx)
{
  std::list<uint> L;
  
  for(uint dataID = start_idx; dataID < _data.size(); ++dataID){
    
    for(auto it = _elemInGraphSigma.begin();it != _elemInGraphSigma.end(); ++it){
      
      uint vertex_id = *it;
      uint point_idx = _id2idx[vertex_id];
      double similarityValue = computeSimilarity(point_idx , dataID);
      
      if ( similarityValue >= _sigma)        
	L.push_back(vertex_id);
    }

    fastInsert(dataID, L);
    //_elemInGraphSigma.push_back(dataID);
    _id2idx[dataID] = dataID;
    L.clear();
  }
}

void oscl::OnlineStarClustering::insertSparseData()
{
  if(_data.size() == 0) // no data to insert
    throw std::runtime_error("\nDataset is empty, nothing to insert.\n");

  std::list<uint> L;
  uint new_point_id = _data.size() - 1;
    
  for(auto &it: _elemInGraphSigma){

    double similarityValue = _data[it].dot( _data[new_point_id]);

    if (similarityValue >= _sigma)        
      L.push_back(it);
  }

  fastInsert(new_point_id, L);
  _elemInGraphSigma.push_back(new_point_id);
  //_id2idx[new_vertex_id] = new_point_idx;
  L.clear();

}

void oscl::OnlineStarClustering::addPoint(Eigen::VectorXd const &p, uint idx)
{
   // data dimension
  int dim  = 0, start_idx = _data.size();
  if(_data.size() == 0)
    dim = p.size();
  else
    dim = _data.back().size();
  
  if (dim != p.size())
    throw "Invalid length of data vector";

  _data.push_back(p);

  updateSimilarityMatrix(start_idx);
  insertLastPoint(idx);
}


// insert last data point into graph
void oscl::OnlineStarClustering::insertLastPoint(uint new_vertex_id)
{
  if(_data.size() == 0) // no data to insert
    return;

  std::list<uint> L;
  uint new_point_idx = _data.size() - 1;

  for(auto it = _elemInGraphSigma.begin();
      it != _elemInGraphSigma.end(); ++it){
      
      uint old_vertex_id = *it;
      uint old_point_idx = _id2idx[old_vertex_id];
      double similarityValue =  computeSimilarity(old_point_idx, _data.size()-1);
      
      if ( similarityValue >= _sigma)        
	L.push_back(old_vertex_id);
  }
  
  fastInsert(new_vertex_id, L);
  _elemInGraphSigma.push_back(new_vertex_id);
  _id2idx[new_vertex_id] = new_point_idx;
 }
