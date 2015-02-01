#include "OnlineStarClustering.h"

uint oscl::OnlineStarClustering::vertexIDMaxDeg(std::list<uint> const &L) const
{
  uint maxVertexID, maxVertexDeg, currVertexID, currVertexDeg;
    
  // Initialize
  currVertexID = *L.begin();
  currVertexDeg = _graph.at(currVertexID).getDegree();
    
  maxVertexID = currVertexID;
  maxVertexDeg = currVertexDeg;
    
  for (auto it = L.begin(); it != L.end(); ++it) {
    
    currVertexID = *it;
    currVertexDeg = _graph.at(currVertexID).getDegree();
    
    if ( currVertexDeg > maxVertexDeg ){
      
      maxVertexID = currVertexID;
      maxVertexDeg = currVertexDeg;
    }
  }
  
  return maxVertexID;
}

void oscl::OnlineStarClustering::sortList(std::list <uint> &AdjCV)
{
  uint alphaID1, alphaID2, swap, i = 0;

  for(auto iter1 = AdjCV.begin(); iter1 != AdjCV.end(); ++iter1){
    alphaID1 = *iter1;

    uint j = 0;

    for (auto iter2 = AdjCV.begin(); j < AdjCV.size() - i - 1; ++iter2){
      alphaID2 = *iter2;

      if (_graph[alphaID1].getDegree() < _graph[alphaID2].getDegree()){
         swap = alphaID2;
         *iter2 = *iter1;
         *iter1 = swap;
      }

      j++;
    }
    i++;
  }
}


