#include "Galaxy.h"

uint oscl::Galaxy::vertexIDMaxDeg(std::set<uint> const &L) const
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

