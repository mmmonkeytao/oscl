#include "LPOSC.h"

#ifdef OPTIMIZE
#include <time.h>
#endif

void oscl::LPOSC::insert1(VectorXd vec, int label)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  _data.push_back(vec);
  // label = -1 if no label
  _labels[_data.size()-1] = static_cast<int>(label);

  // update current datasize
  datasize = _data.size();
  
  // insert into label list
  // check for new labels
  if(labelist.find(label) != labelist.end()){
    labelist[label]++;
  }else{
    labelist[label] = 1;
  }

  // adjacent list
  std::list<uint> L;

#ifdef OPTIMIZE
  clock_t t = clock();
#endif
  // update similarity matrix and sigma graph
  //_sigmaGraph.conservativeResize(_data.size(), _data.size());
  _similarityMatrix.conservativeResize(datasize, datasize);
  _similarityMatrix.bottomRows(1).setZero();
  _similarityMatrix.rightCols(1).setZero();
  
#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for conservative Resize Matrix: "
	    << (float)t / CLOCKS_PER_SEC << std::endl;
  t = clock();
#endif

  uint dataID = datasize-1;  
  // instead of go through all the nodes,
  // we check the most similar centers and
  // also possibly similar centers
  // the possible centers are within lower some
  // percentage from max center

  if( datasize > 1 ){

    const double select_threshold = 0.9995;
    auto centerList = allcenterslist[datasize-2];
    std::map<double, uint, std::greater<double> > max_centers;

    for(auto &cid: centerList)
      {
	max_centers[ computeSimilarity(dataID, cid) ] = cid;	
      }

    auto center_iter = max_centers.begin();
    list<uint> selected_centers;
    uint max_center_id = center_iter->second;
    double max_center_sim = center_iter->first;
    selected_centers.push_back(max_center_id);
    ++center_iter;
   
    while(true)
      {
	if(  center_iter != max_centers.end() ||
	     max_center_sim*select_threshold < center_iter->first )
	  {
	    selected_centers.push_back(center_iter->second);
	    ++center_iter;
	  }
	else
	  break;
      }
    // delete map
    max_centers = std::map<double, uint, std::greater<double> >();

    // create new vertex
    Vertex alpha;
    alpha.setID(dataID);
    alpha.setDomCenterNull();
    alpha.setInQStatus(false);
    _graph.insert(std::pair<uint, Vertex>(dataID, alpha));

    bool gflag1 = true, gflag2 = true;    
    for(auto &c: selected_centers)
      {
        auto domL = _graph[c].getAdjVerticesList();
	double cSim = computeSimilarity(c, dataID);

	bool lflag = true;
        for(auto &domSat: domL)
    	{
    	  double domSim = computeSimilarity(dataID, domSat);

    	  if(domSim*select_threshold > cSim)
	    {
	      _graph[c].deleteAdjVertex(domSat);
	      _graph[c].decrementDegree();
	      _graph[c].deleteDomCenter(domSat);

	      _graph[domSat].deleteAdjCenter(c);
	      _graph[domSat].deleteAdjVertex(c);
	      _graph[domSat].insertAdjVertex(dataID);
	      
	      _graph[dataID].insertAdjVertex(domSat);
	      _graph[dataID].incrementDegree();

	      if(!_graph[domSat].getInQStatus())
		{
		  _graph[domSat].setInQStatus(true);
		  _priorityQ.push(_graph[domSat]);
		}

	      lflag = false;
	      gflag1 = false;
	    }
	  else if(domSim > cSim*select_threshold)
	    {
	      _graph[domSat].incrementDegree();
	      _graph[domSat].insertAdjVertex(dataID);

	      _graph[dataID].incrementDegree();
	      _graph[dataID].insertAdjVertex(domSat);

	      // if(!_graph[domSat].getInQStatus())
	      // 	{
	      // 	  _graph[domSat].setInQStatus(true);
	      // 	  _priorityQ.push(_graph[domSat]);
	      // 	}
	      gflag2 = false;
	    }	  
    	}

    	if(lflag)
	  {
	    _graph[c].incrementDegree();
	    _graph[c].insertAdjVertex(dataID);

	    _graph[dataID].incrementDegree();
	    _graph[dataID].insertAdjVertex(c);
	    _graph[dataID].insertAdjCenter(c);
	    
	    // if(!_graph[c].getInQStatus())
	    //   {
	    // 	_graph[c].setInQStatus(true);
	    // 	_priorityQ.push(_graph[c]);
	    //   }
	    // if(!_graph[dataID].getInQStatus())
	    //   {
	    // 	_graph[dataID].setInQStatus(true);
	    // 	_priorityQ.push(_graph[dataID]);
	    //   }	      
	  }
      }
    if(gflag1 && !gflag2)
      {
	
      }
  }
  // for(uint i = 0; i < datasize-1; ++i){

  //   //double sv = computeSimilarity(i, dataID);
  //   double sv = (_data[i]-_data[dataID]).squaredNorm();

  //   sv = exp(-sv*2.5);
    
  //   if ( sv > _sigma){
  //     _similarityMatrix(i, dataID) = _similarityMatrix(dataID, i) = sv;
  //     //_sigmaGraph.insert(dataID, i) = 1;
  //     L.push_back(i);
  //   }

  // }

  
#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for computing similarities:"
	    << (float)t / CLOCKS_PER_SEC << std::endl;
  t = clock();
#endif

  fastInsert(dataID, L);
  L.clear();

#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for fast Insert:"
	    << (float)t / CLOCKS_PER_SEC << std::endl;
#endif  

#ifdef OPTIMIZE
  t = clock();
#endif
  init_center_star_list();
  update_totedge_graph();

#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for init_center_star list and update edge graph:"
	    << (float)t / CLOCKS_PER_SEC << std::endl;
#endif
}


void oscl::LPOSC::preInitData(VectorXd vec, int label)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  _data.push_back(vec);
  // label = -1 if no label
  _labels[_data.size()-1] = static_cast<int>(label);

  // update current datasize
  datasize = _data.size();
  
  // insert into label list
  // check for new labels
  if(labelist.find(label) != labelist.end()){
    labelist[label]++;
  }else{
    labelist[label] = 1;
  }

  // adjacent list
  std::list<uint> L;
  
  fastInsert(datasize-1, L);
  L.clear();
  
  init_center_star_list();
  update_totedge_graph();  
}
