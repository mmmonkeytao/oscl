#include "LPOSC.h"

/* trial of magnetic online star clusterin */
void oscl::LPOSC::insert2(VectorXd vec, int label, uint iter, double select_threshold)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  /* update dataset */
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
  const uint min_clust_size = 50;

  /* get number of current labels*/
  uint label_number = labelist.size();
  bool attr_flag = label_number * min_clust_size < datasize; 

  /* main algorithm */
  if( datasize > 1 ){

    /* extract best similar centers from the group */
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

    /* get most similar center */
    auto center_iter = max_centers.begin();
    // selected_centers store all centers will be selected
    list<uint> selected_centers;     
    uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    selected_centers.push_back(max_center_id);
    ++center_iter;

    /* compute select rules */
    // select_threshold * (1 - exp(-0.002*iter))
    double current_threshold = select_threshold * (1 - exp(-0.002*iter));
    while(center_iter != max_centers.end())
      {
	if( max_center_sim*current_threshold < center_iter->first )
	  {
	    selected_centers.push_back(center_iter->second);
	    ++center_iter;
	  }
	else
	  break;
      }
    
    /* create new vertex */
    Vertex alpha;
    alpha.setID(dataID);
    alpha.setType(Vertex::SATELLITE);
    alpha.setDomCenterNull();
    alpha.setInQStatus(false);
    alpha.setDegree(0);
    // insert into graph
    _graph.insert(std::pair<uint, Vertex>(dataID, alpha));

    /* loop to go through all sub-stars of selected centers*/
    for(auto &c: selected_centers)
      {
        auto domL = _graph[c].getCopyOfDomSatsList();

	// get current center-new node similarity
	double cSim = _similarityMatrix(c, dataID);

	// compute star-newNode attraction rules
	double attr_threshold = cSim/max_center_sim;
	
	/* attract stars */
        for(auto &domSat: domL)
    	{
	  // compute similarity between new node and stars
    	  double domSim = computeSimilarity(dataID, domSat);
	  _similarityMatrix(dataID, domSat)
	    =_similarityMatrix(domSat, dataID) = domSim;

    	  if(domSim*attr_threshold > cSim)
	    {
	      // if rule satisfied, separate this node
	      // from original center
	      _graph[c].decrementDegree();
	      _graph[c].deleteDomSats(domSat);

	      // connect to new node
	      _graph[domSat].setDomCenter(dataID);

	      // insert new stars
	      _graph[dataID].insertDomSats(domSat);
	      _graph[dataID].incrementDegree();
	      
	    }
	} // end of each dominate satellites list

	// set criteria of the current center
	// either stay as it is
	// or to be merged to other bigger clusters
	if( attr_flag && (_graph[c].getDegree()+1) < min_clust_size)
	  {
	    // erase from centerlist and max_center list
	    current_centerList.remove(c);
	    _graph[c].setType(Vertex::SATELLITE);
	    max_centers.erase(_similarityMatrix(dataID, c));

	    // then it together with all its stars will be attracted
	    // to closest cluster
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
	    /* check for new node */
	    if(max < _similarityMatrix(c, dataID))
	      {
		max = _similarityMatrix(c, dataID);
		maxID = dataID;
	      }

	    // connect to maxmum similar cluster
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

    if( attr_flag && ((_graph[dataID].getDegree()+1) < min_clust_size) && max_centers.size() > 0)
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
  //update_totedge_graph();

}

