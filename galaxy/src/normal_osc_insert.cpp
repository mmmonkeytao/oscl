#include "Galaxy.h"

void oscl::Galaxy::normal_osc_insert(Eigen::VectorXd vec, uint dataID, int label, uint iter, double select_threshold)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  /* store into dataset */
  incrementDataSize();
  _dataIDs[iter] = dataID;
  _labels[dataID] = label;
  _dataXXL.push_back(std::move(vec));

  /* add labels */
  if(_labelist.find(label) != _labelist.end()){
    _labelist[label]++;
  }else{
    _labelist[label] = 1;
  }

  // adjacent list
  std::list<uint> L;

  // cluster evaluation
  // initialize the var
  _center_star_changed = 0;

  /* attract stars */
  for(uint i = 0; i < _datasize-1; ++i)
    {
      // compute similarity between new node and stars
      uint cID = _dataIDs[i];
      double sim = computeSimilarity(dataID, cID);

      if(sim > select_threshold){	      
	L.push_back(cID);
      }
    } // end of each dominate satellites list

  oscInsert(dataID, L);
  L.clear();

  _cs_changed.push_back(_center_star_changed);
  

}
