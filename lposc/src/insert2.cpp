#include "LPOSC.h"

/* trial of magnetic online star clusterin */

void oscl::LPOSC::insert2(VectorXd vec, int label, uint iter, double select_threshold)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  _data.push_back(vec);

  // label = -1 if no label
  _labels[_data.size()-1] = label;

  // update current datasize
  datasize = _data.size();
  
  // insert into label list
  // check for new labels
  if(labelist.find(label) != labelist.end()){
    labelist[label]++;
  }else{
    labelist[label] = 1;
  }

  // update similarity matrix and sigma graph
  _similarityMatrix.conservativeResize(datasize, datasize);
  _similarityMatrix.bottomRows(1).setZero();
  _similarityMatrix.rightCols(1).setZero();
  
  uint dataID = datasize-1;
  const uint min_clust_size = 5;

  if( datasize > 1 ){

    auto centerList = allcenterslist[datasize-2];
    list<uint> current_centerList;
    std::map<double, uint, std::greater<double> > max_centers;

    for(auto &cid: centerList)
      {
	double svar = computeSimilarity(dataID, cid);
	current_centerList.push_back(cid);
	
	_similarityMatrix(dataID, cid)
	  = _similarityMatrix(cid, dataID) = svar;
	max_centers[svar] = cid;	
      }

    auto center_iter = max_centers.begin();
    list<uint> selected_centers;     
    uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    selected_centers.push_back(max_center_id);
    ++center_iter;

    while(center_iter != max_centers.end())
      {
	if( max_center_sim*select_threshold < center_iter->first )
	  {
	    
	    selected_centers.push_back(center_iter->second);
	    ++center_iter;
	  }
	else
	  break;
      }
    
    // create new vertex
    Vertex alpha;
    alpha.setID(dataID);
    alpha.setType(Vertex::SATELLITE);
    alpha.setDomCenterNull();
    alpha.setInQStatus(false);
    alpha.setDegree(0);
    
    _graph.insert(std::pair<uint, Vertex>(dataID, alpha));

    for(auto &c: selected_centers)
      {
        auto domL = _graph[c].getCopyOfDomSatsList();
	double cSim = _similarityMatrix(c, dataID);

	// it starts to attract stars
        for(auto &domSat: domL)
    	{
    	  double domSim = computeSimilarity(dataID, domSat);
	  _similarityMatrix(dataID, domSat)
	    =_similarityMatrix(domSat, dataID) = domSim;

    	  if(domSim*pow(select_threshold, 2) > cSim)
	    {

	      _graph[c].decrementDegree();
	      _graph[c].deleteDomSats(domSat);

	      _graph[domSat].setDomCenter(dataID);
	      
	      _graph[dataID].insertDomSats(domSat);
	      _graph[dataID].incrementDegree();
	      
	    }
	} // end of each dominate satellites list

	
	if(_graph[c].getDegree() < min_clust_size)
	  {
	    current_centerList.remove(c);
	    _graph[c].setType(Vertex::SATELLITE);
	    max_centers.erase(_similarityMatrix(dataID, c));
	    
	    double max = -1.0;
	    uint maxID = 0;
	    for(auto &center: current_centerList){
	      
	      double svar = computeSimilarity(c, center);
	      _similarityMatrix(c, center) =
		_similarityMatrix(c, center) = svar;

	      if(svar > max)
		{
		  max = svar;
		  maxID = center;
		}
	    }
	    if(max < _similarityMatrix(c, dataID))
	      {
		max = _similarityMatrix(c, dataID);
		maxID = dataID;
	      }

	    for(auto &sats: _graph[c].getDomSatsList())
	      {
		_graph[maxID].incrementDegree();
		_graph[maxID].insertDomSats(sats);
		_graph[sats].setDomCenter(maxID);
	      }

	    _graph[c].setDegree(0);
	    _graph[maxID].incrementDegree();
	    _graph[maxID].insertDomSats(c);
	    _graph[c].setDomCenter(maxID);
	    _graph[c].clearDomSatsList();
	    
	  }
        
      }// end of all selected centers 

    if(_graph[dataID].getDegree() < min_clust_size && current_centerList.size() > 0)
      {

	uint maxID = (max_centers.begin())->second;
	for(auto &sats: _graph[dataID].getDomSatsList())
	  {
	    _graph[maxID].incrementDegree();
	    _graph[maxID].insertDomSats(sats);
	    _graph[sats].setDomCenter(maxID);
	  }

	_graph[dataID].setDegree(0);
	_graph[maxID].incrementDegree();
	_graph[maxID].insertDomSats(dataID);
	_graph[dataID].setDomCenter(maxID);
	_graph[dataID].clearDomSatsList();

      }
    else
      {
	_graph[dataID].setType(Vertex::CENTER);
      }
    
  }
  else
    {
      // create new vertex
      Vertex alpha;
      alpha.setID(dataID);
      alpha.setType(Vertex::CENTER);
      alpha.setDomCenterNull();
      //alpha.setInQStatus(false);
      alpha.setDegree(0);
      _graph.insert(std::pair<uint, Vertex>(dataID, alpha));
    }
  
  init_center_star_list();
  update_totedge_graph();

}

