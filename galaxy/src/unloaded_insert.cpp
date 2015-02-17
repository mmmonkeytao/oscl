#include "Galaxy.h"

void oscl::Galaxy::unloaded_insert(uint dataID, int label, uint iter, double select_threshold)
{
  incrementDataSize();
  
  _dataIDs[iter] = dataID;
  
  _labels[dataID] = label;

  if(_labelist.find(label) != _labelist.end()){
    _labelist[label]++;
  }else{
    _labelist[label] = 1;
  }

  /* main algorithm */
  if( _datasize > 1 ){

    // load current ID's fea
    load_fea_pool(dataID);
    // load center features
    load_fea_pool(_centerList);

    const uint min_clust_size = (uint)ceil(100.0 * (1.0 - exp(-0.0008*iter)));
    bool attr_flag = min_clust_size*51 < _datasize;
    
    /* extract best similar centers from the group */
    // to sort current centers
    std::map<double, uint, std::greater<double> > max_centers;

    for(auto &cid: _centerList)
      {
	double svar = loaded_computeSimilarity(dataID, cid);
	max_centers[svar] = cid;	
      }

    /* get most similar center */
    auto center_iter = max_centers.begin();
    //uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    ++center_iter;
    auto select_iter_end = center_iter;
    
    /* compute select rules */
    // select_threshold * (1 - exp(-0.0005*iter))
    double current_threshold = max_center_sim * select_threshold * (1 - exp(-0.0005*iter));
    
    while(select_iter_end != max_centers.end())
      {
	if( current_threshold < select_iter_end->first )
	  {
	    ++select_iter_end;
	  }
	else
	  break;
      }

    // load dominating stars
    load_fea_pool(max_centers, select_iter_end);
    
    /* create new vertex */
    Planet alpha(dataID);
    
    // insert into graph
    _graph.insert(std::pair<uint, Planet>(dataID, alpha));

    /* loop to go through all sub-stars of selected centers*/
    auto cIter = max_centers.begin();
    while(cIter != select_iter_end)
      {
        auto domL = _graph[cIter->second].getCopyOfDomSatsList();

	// get current center-new node similarity
	double cSim = cIter->first;

	// compute star-newNode attraction rules
	double attr_threshold = cSim/max_center_sim;
	
	/* attract stars */
        for(auto &domSat: domL)
	  {
	    // compute similarity between new node and stars
	    double domSim = loaded_computeSimilarity(dataID, domSat);

	    if(domSim*attr_threshold > cSim)
	      {
		// if rule satisfied, separate this node
		// from original center
		_graph[cIter->second] >> _graph[domSat];
		_graph[dataID] << _graph[domSat];

	      }
	  } // end of each dominate satellites list

	// set criteria of the current center
	// either stay as it is
	// or to be merged to other bigger clusters
	if( attr_flag && (_graph[cIter->second].getDegree()+1) < min_clust_size)
	  {
	    // erase from centerlist and max_center list
	    _centerList.erase(cIter->second);
	    _graph[cIter->second].setType(Planet::STAR);
	    max_centers.erase(cSim);

	    // then it together with all its stars will be attracted
	    // to closest cluster
	    double max = -1.0;
	    uint maxID = 0;
	    for(auto &center: _centerList){
	      
	      double svar = loaded_computeSimilarity(cIter->second, center);

	      if(svar > max)
		{
		  max = svar;
		  maxID = center;
		}
	    }
	    /* check for new node */
	    if(max < cSim)
	      {
		max = cSim;
		maxID = dataID;
	      }

	    // connect to maxmum similar cluster
	    for(auto &sats: _graph[cIter->second].getDomSatsList())
	      {
		_graph[maxID] << _graph[sats];		
	      }

	    _graph[cIter->second].setDegree(0);
	    _graph[maxID].incrementDegree();
	    _graph[maxID].insertDomSats(cIter->second);
	    _graph[cIter->second].setDomCenter(maxID);
	    _graph[cIter->second].clearDomSatsList();
	  }

	++cIter;
	
      }// end of all selected centers 

    if( attr_flag && ((_graph[dataID].getDegree()+1) < min_clust_size) && max_centers.size() > 0)
      {

	uint maxID = (max_centers.begin())->second;
	for(auto &sats: _graph[dataID].getDomSatsList())
	  {
	    _graph[maxID] << _graph[sats];
	  }

	_graph[dataID].setDegree(0);
	_graph[maxID].incrementDegree();
	_graph[maxID].insertDomSats(dataID);
	_graph[dataID].setDomCenter(maxID);
	_graph[dataID].clearDomSatsList();

      }
    else
      {
	_graph[dataID].setType(Planet::CENTER);
	_centerList.insert(dataID);
      }

    set_fea_pool_null();
  }
  else
    {
      // create new vertex
      Planet alpha(dataID);
      alpha.setType(Planet::CENTER);
      _graph.insert(std::pair<uint, Planet>(dataID, alpha));
      _centerList.insert(dataID);
    }
    
}
