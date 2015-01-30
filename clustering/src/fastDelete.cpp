#include "OnlineStarClustering.h"

void oscl::OnlineStarClustering::deleteData(std::list<uint> ID)
{  
  for(std::list<uint>::iterator it = ID.begin(); it != ID.end(); ++it){
    fastDelete(*it);
    
    _graph.erase(*it);
    _elemInGraphSigma.remove(*it);
    _id2idx.erase(*it);

  }  
}

void oscl::OnlineStarClustering::fastDelete(uint alphaID){

  uint betaID, betaDomCenterID;
  uint gammaID, nuID; //DomCenterID;
  std::list<uint> AdjVL = _graph[alphaID].getAdjVerticesList();
  std::list<uint> DomSL, AdjCL;

  for (auto it = AdjVL.begin(); it != AdjVL.end(); ++it){
        betaID = *it;
        _graph[betaID].decrementDegree();
        _graph[betaID].deleteAdjVertex(alphaID);

        if (!_graph[betaID].isDomCenterNull()){
          betaDomCenterID = _graph[betaID].getDomCenter();
          _graph[betaDomCenterID].getDomSatsList().remove(alphaID);
	  
          // for (list<uint>::iterator iter1 = DomSL.begin(); iter1 != DomSL.end(); ++iter1){
          //   DomCenterID = *iter1;
          //   AdjCL = _graph[DomCenterID].getAdjCentersList();
          //   sortList(AdjCL);
          // }
        } 
  }

  if(_graph[alphaID].getType() == Vertex::SATELLITE){

    AdjCL = _graph[alphaID].getAdjCentersList();

      for (auto iter1 = AdjCL.begin(); iter1 != AdjCL.end(); ++iter1){
	
        betaID = *iter1;
	
        if(betaID == _graph[alphaID].getDomCenter()){
          (_graph[betaID].getDomSatsList()).remove(alphaID);
        } 

        if( !_graph[betaID].isDomSatsListEmpty() ){
            gammaID = vertexIDMaxDeg(_graph[betaID].getDomSatsList());
	    
            while(!_graph[betaID].isDomSatsListEmpty() && 
                _graph[gammaID].getDegree() > _graph[betaID].getDegree()){

                _graph[betaID].getDomSatsList().remove(gammaID);
                _graph[gammaID].setDomCenter(-1);

                if(!_graph[gammaID].getInQStatus()){
                  _graph[gammaID].setInQStatus(true);
                  _priorityQ.push(_graph[gammaID]);
                }
		
		if(_graph[betaID].getDomSatsList().size() > 0)
		  gammaID = vertexIDMaxDeg(_graph[betaID].getDomSatsList());	      
            }//end while
        }
      }//end for 
  } else {

    for(auto iter1 = AdjVL.begin(); iter1 != AdjVL.end(); ++iter1){
      betaID = *iter1;
      _graph[betaID].getAdjCentersList().remove(alphaID);
    }

    DomSL = _graph[alphaID].getDomSatsList();

    if(DomSL.size() > 0){
      for(auto iter1 = DomSL.begin(); iter1 != DomSL.end(); ++iter1){
	nuID = *iter1;
	_graph[nuID].setDomCenter(-1);

	if(!_graph[nuID].getInQStatus()){
	  _graph[nuID].setInQStatus(true);	          
	  _priorityQ.push(_graph[nuID]);
	}	
      }
    }

  }
  
  fastUpdate(alphaID);
}
