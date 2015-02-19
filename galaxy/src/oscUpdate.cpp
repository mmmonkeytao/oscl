#include "Galaxy.h"

void oscl::Galaxy::oscUpdate(uint alphaID)
{
    uint phiID, deltaID, muID, lambdaID, vID, domCenterIDForPhi;
    
    Planet topPriorityQ;
    
    while(!_priorityQ.empty()){
      
      topPriorityQ = _priorityQ.top();
      phiID = topPriorityQ.getID();
      _priorityQ.pop();
      
      if (_graph[phiID].isAdjCentersListEmpty()){
	
	// Set promoted to center.
	_graph[phiID].setType(Planet::CENTER); 

	/////////////////////
	// cluster evaluation
	/////////////////////
	++_center_star_changed;
	/////// end /////////

	//
	_centerList.insert(phiID);
	//
	
	// CHANGE FOR FAST VERSION.            
	// Promoted to center. It will not have a dom center. 
	_graph[phiID].setDomCenterNull();
        
	// Record stats.
	//_perIterSatellitePromotions[alphaID]++;
	//_perIterNumClusters[alphaID]++;
        
	for (auto &iter: _graph[phiID].getAdjPlanetsList())	  
	  _graph[iter].insertAdjCenter(phiID);
	
      }
      else {
	
	lambdaID = vertexIDMaxDeg(_graph[phiID].getAdjCentersList());
        
	if (_graph[lambdaID].getDegree() >= _graph[phiID].getDegree()){
	  
	  // If phi has a dom center then correct the dom center's list.
	  if (!_graph[phiID].isDomCenterNull()){
	    
	    domCenterIDForPhi = _graph[phiID].getDomCenter();
	    _graph[domCenterIDForPhi].deleteDomSats(phiID);
	  }
	  /////////////////////
	  // cluster evaluation
	  /////////////////////
	  ++_center_star_changed;
	  /////// end /////////

	  // If phi does not have a dom center then make lambda as its dom center.
	  _graph[phiID].setDomCenter(lambdaID);
	  _graph[lambdaID].insertDomSats(phiID);
	}
	
	else {
	  /////////////////////
	  // cluster evaluation
	  /////////////////////
	  ++_center_star_changed;
	  /////// end /////////

	  _centerList.insert(phiID);

	  // Make phi center.
	  _graph[phiID].setType(Planet::CENTER);
	  _graph[phiID].setDomCenterNull();
          
	  // Record stats.
	  //_perIterSatellitePromotions[alphaID]++;
	  //_perIterNumClusters[alphaID]++;
          
	  for (auto &iter2:_graph[phiID].getAdjPlanetsList()){
	    _graph[iter2].insertAdjCenter(phiID);
	  }
          
	  // Get a copy, otherwise inner loop can modify the list. 
	  for (auto &iter3: _graph[phiID].getCopyOfAdjCentersList()){               
	    deltaID = iter3;
	    /////////////////////
	    // cluster evaluation
	    /////////////////////
	    ++_center_star_changed;
	    /////// end /////////

	    _centerList.erase(deltaID);
	    
	    _graph[deltaID].setType(Planet::STAR); // Broken star.
	    _graph[deltaID].setDomCenter(phiID);    
	    _graph[phiID].insertDomSats(deltaID); // Add deltaID to phiID's dom center list.
	    
	    //_perIterStarsBroken[alphaID]++;         // Record broken star.
	    //_perIterNumClusters[alphaID]--;         // Record dec in num of clusters.
		
	    for (auto &innerIter: _graph[deltaID].getAdjPlanetsList()){
	      muID = innerIter;
	      _graph[muID].deleteAdjCenter(deltaID);

	      /////////////////////
	      // cluster evaluation
	      /////////////////////
	      ++_center_star_changed;
	      /////// end /////////
	    }                        
	    
	    for (auto &innerIter: _graph[deltaID].getDomSatsList()){
	      
	      vID = innerIter;
	      _graph[vID].setDomCenterNull();
              
	      if (_graph[vID].getInQStatus() == false){
		
		_graph[vID].setInQStatus(true);
		_priorityQ.push(_graph[vID]);
	      }
	    }                        
            
	    // Clear delta's dom centers list.
	    _graph[deltaID].clearDomSatsList();
	  }
	  
	  // Clear phi's center list.
	  _graph[phiID].clearCentersList();                
	}
      }
      
      _graph[phiID].setInQStatus(false);
      
    }// While ends. 
} // Function ends.

