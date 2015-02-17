#include "Galaxy.h"

void oscl::Galaxy::loaded_insert_noSim(Eigen::VectorXd vec, uint dataID, int label, uint iter, double select_threshold)
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

  if(_datasize > 1){
    
    const uint min_clust_size
      = (uint)ceil(_min_cluster_size_wanted * (1.0 - exp(-_clust_size_eps*iter)));

    bool attr_flag = min_clust_size*_labelist.size() < _datasize;
    //bool attr_flag = 500 < _datasize;
    
    /* extract best similar centers from the group */
    // to sort current centers
    std::map<double, uint, std::greater<double> > max_centers;

    for(auto &cid: _centerList)
      {
	double svar = computeSimilarity(cid, dataID);
	max_centers[svar] = cid;	
      }

    /* get most similar center */
    auto center_iter = max_centers.begin();
    //uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    ++center_iter;
    // iterator for choosing the end
    // where stops for selected centers
    auto select_iter_end = center_iter;

    /* compute select rules */
    // select_threshold * (1 - exp(-0.0005*iter))
    double current_threshold
      = max_center_sim * select_threshold * (1.0 - exp(-_threshold_eps*iter));

    /////////////////////////////
    uint counter_center_check = 1;
    /////////////////////////////
    while(select_iter_end != max_centers.end())
      {
	if( current_threshold < select_iter_end->first )
	  {
	    ++select_iter_end;
	    //////////////////////
	    ++counter_center_check;
	    //////////////////////
	  }
	else
	  break;
      }

    ////////////////////////////////////////////////
    center_check_num.push_back(counter_center_check);
    /////////////////////////////////////////////////

    /////////////////////////////
    uint counter_star_broken = 0;
    /////////////////////////////
    
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
	uint cID = cIter->second;

	// compute star-newNode attraction rules
	double attr_threshold = cSim/max_center_sim;

	/* attract stars */
        for(auto &domSat: domL)
	  {
	    // compute similarity between new node and stars
	    double domSim = computeSimilarity(dataID, domSat);

	    if(domSim*attr_threshold > cSim)
	      {
		// if rule satisfied, separate this node
		// from original center
		_graph[cID] >> _graph[domSat];
		_graph[dataID] << _graph[domSat];

		////////////////////////
		++counter_star_broken;
		///////////////////////
	      }

	  } // end of each dominate satellites list

	++cIter;
	    // set criteria of the current center
	    // either stay as it is
	    // or to be merged to other bigger clusters
	if( attr_flag && (_graph[cID].getDegree()+1) < min_clust_size)
	  {
	    // erase from centerlist and max_center list
	    _centerList.erase(cID);
	    _graph[cID].setType(Planet::STAR);
	    max_centers.erase(cSim);

	    // then it together with all its stars will be attracted
	    // to closest cluster
	    double max = -1.0;
	    uint maxID = 0;
	    
	    for(auto &center: _centerList){  
	      double svar = computeSimilarity(cID, center);

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
	    for(auto &sats: _graph[cID].getDomSatsList())
	      {
		_graph[maxID] << _graph[sats];

		//////////////////////
		++counter_star_broken;
		//////////////////////
	      }

	    _graph[maxID].incrementDegree();
	    _graph[maxID].insertDomSats(cID);
	    _graph[cID].setDegree(0);
	    _graph[cID].setDomCenter(maxID);
	    _graph[cID].clearDomSatsList();		
	  }
	
      }// end of all selected centers

    if( attr_flag && ((_graph[dataID].getDegree()+1) < min_clust_size) && max_centers.size() > 0)
      {
	uint maxID = (max_centers.begin())->second;

	for(auto &sats: _graph[dataID].getDomSatsList())
	  {
	    _graph[maxID] << _graph[sats];
	  }

	_graph[maxID].incrementDegree();
	_graph[maxID].insertDomSats(dataID);
	_graph[dataID].setDegree(0);
	_graph[dataID].setDomCenter(maxID);
	_graph[dataID].clearDomSatsList();
	_graph[dataID].setType(Planet::STAR);
      }
    else
      {
	_graph[dataID].setType(Planet::CENTER);
	_centerList.insert(dataID);
      }

    star_broken_num.push_back(counter_star_broken);

  } else {
    // create new vertex
    Planet alpha(dataID);
    alpha.setType(Planet::CENTER);
    _graph.insert(std::pair<uint, Planet>(dataID, alpha));
    _centerList.insert(dataID);

    // performance analysis
    center_check_num.push_back(0);
    star_broken_num.push_back(0);
  }

}





