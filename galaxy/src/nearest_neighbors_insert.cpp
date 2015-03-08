#include "Galaxy.h"

bool oscl::Galaxy::nearest_neighbors_insert(Eigen::VectorXd vec, uint dataID, int label, uint iter, double select_threshold)
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
    
    const uint min_clust_size = (uint)ceil(_min_cluster_size_wanted * (1.0 - exp(-_clust_size_eps*iter) + 1.0e-52));
    //const uint min_clust_size = 15; //_min_cluster_size_wanted;
    
    //bool attr_flag = min_clust_size*_labelist.size() < _datasize;
    //bool attr_flag = 20 * min_clust_size < _datasize;
    bool attr_flag = min_clust_size * _centerList.size() < _datasize;

    /* extract best similar centers from the group */
    // to sort current centers
    std::map<double, uint, std::less<double> > max_centers;

    for(auto &cid: _centerList)
      {
	double svar = computeSimilarity(cid, dataID);
	max_centers[svar] = cid;	
      }

    /* get most similar center */
    auto center_iter = max_centers.begin();
    uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    ++center_iter;

    // iterator for choosing the end
    // where stops for selected centers
    auto select_iter_end = center_iter;

    /* compute select rules */
    // select_threshold * (1 - exp(-0.0005*iter))
    double current_threshold
      = max_center_sim / (select_threshold * (1.0 - exp(-_threshold_eps*iter) + 1.0e-52));

    /////////////////////////////
    uint counter_center_check = 1;
    /////////////////////////////

    std::map<uint, std::list<uint> > CSL;
    while(select_iter_end != max_centers.end())
      {
	if( current_threshold > select_iter_end->first )
	  {
	    CSL.insert(std::pair<uint, std::list<uint> >
		       (select_iter_end->second, std::list<uint>() ));
	    
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
    
    /* create new vertex */
    Planet alpha(dataID);
    // insert into graph
    _graph.insert(std::pair<uint, Planet>(dataID, alpha));

    /* loop to go through all sub-stars of selected centers*/
    auto cIter = max_centers.begin();

    //
    uint near_star_counter = 0;
    
    while(cIter != select_iter_end)
      {
        auto domL = _graph[cIter->second].getCopyOfDomSatsList();
	double domSim = cIter->first;
	double attr_threshold = domSim/max_center_sim;

	/* attract stars */
        for(auto &domSat: domL)
	  {
	    // compute similarity between new node and stars
	    double domSim = computeSimilarity(dataID, domSat);
	    double domC = _graph[domSat].getAttr(); 
	    
	    if(domSim*attr_threshold < domC)//max_center_sim)
	      {
		CSL[cIter->second].push_back(domSat);

		near_star_counter++;
	      }

	  } // end of each dominate satellites list

	++cIter;
      }// end of all selected centers

    /////////////////////////////
    uint counter_star_broken = 0;
    /////////////////////////////

    if(attr_flag && near_star_counter+1 > min_clust_size){

      _graph[dataID].setType(Planet::CENTER);

      for(auto &center: CSL){

	for(auto &domSat: center.second){
	  _graph[center.first] >> _graph[domSat];
	  _graph[dataID] << _graph[domSat];

	  double satSim = computeSimilarity(dataID, domSat);
	  _graph[domSat].setAttr(satSim);

	  ////////////////////////
	  ++counter_star_broken;
	  ///////////////////////
	}
	
      }
      _centerList.insert(dataID);
      
      for(auto &center: CSL){

      	if(_graph[center.first].getDegree() + 1 < min_clust_size){

      	  _centerList.erase(center.first);
      	  _graph[center.first].setType(Planet::STAR);

      	  uint maxID;
      	  double sim = 1.0e5;

	  for(auto &ccenter: _centerList){
      	    double tmpsim = computeSimilarity(ccenter, center.first);
      	    if(tmpsim < sim){
      	      maxID = ccenter;
      	      sim = tmpsim;
      	    }
      	  }
	  
      	  _graph[maxID] << _graph[center.first];
	  _graph[center.first].setAttr(sim);
	  
	  for(auto &domSat: _graph[center.first].getCopyOfDomSatsList()){

	    double satSim = computeSimilarity(domSat, maxID);
	    
      	    _graph[maxID] << _graph[domSat];
      	    _graph[center.first] >> _graph[domSat];
	    _graph[domSat].setAttr(satSim);
	  }
	
      }
      // for(auto &center: CSL){

      // 	if(_graph[center.first].getDegree() + 1 < min_clust_size){

      // 	  _centerList.erase(center.first);
      // 	  _graph[center.first].setType(Planet::STAR);

      // 	  uint maxID;
      // 	  double sim = 1.0e5;
      // 	  for(auto &ccenter: _centerList){
      // 	    double tmpsim = computeSimilarity(ccenter, center.first);
      // 	    if(tmpsim < sim){
      // 	      maxID = ccenter;
      // 	      sim = tmpsim;
      // 	    }
      // 	  }
      // 	  _graph[maxID] << _graph[center.first];
	  
      // 	  for(auto &domSat: _graph[center.first].getCopyOfDomSatsList()){

      // 	    uint maxID; 
      // 	    double sim = 1.0e5;
	    
      // 	    for(auto &ccenter: _centerList){

      // 	      double tmpsim = computeSimilarity(ccenter, domSat);

      // 	      if(tmpsim < sim){
      // 		maxID = ccenter;
      // 		sim = tmpsim;
      // 	      }
      // 	    }

      // 	    _graph[maxID] << _graph[domSat];
      // 	    _graph[center.first] >> _graph[domSat];
      // 	  }
      // 	}
      }
    } else {

      _graph[max_center_id] << _graph[dataID];
      _graph[dataID].setAttr(max_center_sim);
    }

    center_check_num.push_back(counter_center_check);
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

    // require_label.push_back(1);
  }

  if(_graph[dataID].getType() == Planet::CENTER)
    return 1;
  else
    return 0;

}





