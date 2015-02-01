#include "OnlineStarClustering.h"

void oscl::OnlineStarClustering::fastInsert(uint alphaID, std::list <uint> &L)
{    
  // Make a vertex and insert in graph sigma.
  Vertex alpha;
  alpha.setID(alphaID);
    
  alpha.setDomCenterNull();
  alpha.setInQStatus(false);
    
  _graph.insert(std::pair<uint, Vertex>(alphaID ,alpha));
    
  uint betaID, betaDomCenterID, alphaDomCenterID;
    
  // For all beta in list L.
  for (auto &it: L){
    
    betaID = it;

    // Increment degrees
    _graph[alphaID].incrementDegree();
    _graph[betaID].incrementDegree();
    
    // Insert alphaID and betaID in each other's adjacency lists.
    _graph[alphaID].insertAdjVertex(betaID);
    _graph[betaID].insertAdjVertex(alphaID);
    
    ///////
    // list<uint> DomSL, AdjCL;
    // uint DomCenterID;

    // if (!_graph[betaID].isDomCenterNull()){
    //   betaDomCenterID = _graph[betaID].getDomCenter();
    //   DomSL = _graph[betaDomCenterID].getDomSatsList();

    //   for (auto &iter1: DomSL){
    //     DomCenterID = *iter1;
    //     AdjCL = _graph[DomCenterID].getAdjCentersList();
    //     sortList(AdjCL);
    //   }
    // }  
    ///////

    // Update center adjacency list if beta was a center.
    if (_graph[betaID].getType() == Vertex::CENTER) {
      _graph[alphaID].insertAdjCenter(betaID);
    }
    else {
      // **** CHANGE FOR FAST VERSION ***
      // Get degree of beta's dominant center
      betaDomCenterID = _graph[betaID].getDomCenter();
      
      // Insert if deg of beta has exceeded the one of its dom center.
      if ( _graph[betaID].getDegree() > _graph[betaDomCenterID].getDegree() ) {
	
	// Insert beta into the priority queue.
	_graph[betaID].setInQStatus(true);
	_priorityQ.push(_graph[betaID]);
      }    
    }
  } // List iteration ends.
  
  
  // **** CHANGE FOR FAST VERSION ***
    
  // If alpha's adjacent list is empty
  if (_graph[alphaID].isAdjCentersListEmpty()){
    
    // Insert alpha into the priority queue.
    _graph[alphaID].setInQStatus(true);
    _priorityQ.push(_graph[alphaID]); 
  }
    
  else {
    // Find alpha's dominant center.
    alphaDomCenterID = vertexIDMaxDeg(_graph[alphaID].getAdjCentersList());
    
    // Assign alpha's dominant center.
    _graph[alphaID].setDomCenter(alphaDomCenterID);
    
    // Insert alphaID into alpha's dom center's domsats.
    _graph[alphaDomCenterID].insertDomCenter(alphaID);
    
    // If alpha's degree exceeds alpha's dom center's degree.
    if ( _graph[alphaID].getDegree() > _graph[alphaDomCenterID].getDegree() ) {
      
      // Insert alpha into the priority queue.
      _graph[alphaID].setInQStatus(true);
      _priorityQ.push(_graph[alphaID]);
    }   
  }
          
  // Update using priority queue.
  fastUpdate(alphaID);
}
