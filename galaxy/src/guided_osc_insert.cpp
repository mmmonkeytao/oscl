#include "Galaxy.h"

using namespace std;

void oscl::Galaxy::guided_osc_insert(Eigen::VectorXd vec, uint dataID, int label, uint iter, double select_threshold)
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
  //std::cout << "iter: " << iter << std::endl;
  // adjacent list
  std::list<uint> L;

  if(_datasize >= 400){
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
    uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    ++center_iter;
    // iterator for choosing the end
    // where stops for selected centers
    auto select_iter_end = center_iter;

    /* compute select rules */
    // select_threshold * (1 - exp(-0.0005*iter))
    //double current_threshold
    //  = max_center_sim * select_threshold;// * (1.0 - exp(-_threshold_eps*iter));
    //std::cout << max_center_sim << "  " << _labels[max_center_id] <<"  "
    //<< label ;

    double current_threshold = max_center_sim * select_threshold;
    
    /////////////////////////////
    uint counter_center_check = 1;
    /////////////////////////////
    while(select_iter_end != max_centers.end())
      {
	if( current_threshold < select_iter_end->first )
	  {
	    //std::cout << current_threshold << "  "  << _labels[select_iter_end->second] <<"  ";	    
	    ++select_iter_end;
	    //////////////////////
	    ++counter_center_check;
	    //////////////////////
	  }
	else
	  break;
      }
    //std::cout << "\n";
    ////////////////////////////////////////////////
    center_check_num.push_back(counter_center_check);
    /////////////////////////////

    /* loop to go through all sub-stars of selected centers
     * select for more similar planet
     */
    std::map<double, uint, std::greater<double> > max_planets;
    auto cIter = max_centers.begin();
  
    while(cIter != select_iter_end)
      {
	//auto domL = _graph[cIter->second].getCopyOfDomSatsList();
	auto domL =  _graph[cIter->second].getCopyOfAdjPlanetsList();
	
	// get current center-new node similarity
	double cSim = cIter->first;
	uint cID = cIter->second;

	max_planets[cSim] = cID;
	
	// compute star-newNode attraction rules
	//double attr_threshold = cSim/max_center_sim;

	//uint counter = 0; // count number of stars more similar to new node
	
	/* attract stars */
	for(auto &domSat: domL)
	  {
	    // compute similarity between new node and stars
	    double domSim = computeSimilarity(dataID, domSat);

	    max_planets[domSim] = domSat;
	    //L.push_back(domSat);
	    //if(domSim*select_threshold > current_threshold){
	    
	    //++counter;
	    //L.push_back(domSat);
	      //}
	    //	    else if(domSim > max_center_sim * select_threshold){
	    //else if(domSim > current_threshold){
	    //  L.push_back(domSat);
	    //}

	    // if(domSim > current_threshold){
	    //   L.push_back(domSat);
	    // }

	  } // end of each dominate satellites list

	++cIter;

	//if((double)counter <= 0.3 * (double)domL.size() )
	  {
	    //L.push_back(cID);
	  }
      }// end of all selected centers

    auto max_planet_iter = max_planets.begin();
    double max = max_planet_iter->first;
    uint maxID = max_planet_iter->second;
    ++max_planet_iter;
    L.push_back(maxID);
    //cout << label << " " << max << " " << _labels[maxID] << "       ";
    while(max_planet_iter != max_planets.end())
      {
	if(max_planet_iter->first > max  * select_threshold)
	  { //cout << max_planet_iter->first <<" " << _labels[max_planet_iter->second] << "     ";
	    L.push_back(max_planet_iter->second);
	  }
	else
	  break;
	
	++max_planet_iter;
      }
    //cout <<"\n";
  }
  else {
      _centerList.insert(dataID);
      center_check_num.push_back(0);
  }
  
  _center_star_changed = 0;
    
  oscInsert(dataID, L);
  L.clear();

  _cs_changed.push_back(_center_star_changed);
    
}
