#include "LPOSC.h"

#ifdef OPTIMIZE
#include <time.h>
#endif

void oscl::LPOSC::insert(VectorXd vec, int label)
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
  for(uint i = 0; i < datasize-1; ++i){

    double sv = computeSimilarity(i, dataID);

    if ( sv > _sigma){
      _similarityMatrix(i, dataID) = _similarityMatrix(dataID, i) = sv;
      //_sigmaGraph.insert(dataID, i) = 1;
      L.push_back(i);
    }

  }

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


void oscl::LPOSC::insert(SpVec vec, int label)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  SpData.push_back(vec);
  // label = -1 if no label
  _labels[SpData.size()-1] = static_cast<int>(label);

  // update datasize
  datasize = SpData.size();
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
  simMat.conservativeResize(datasize,datasize);
  
#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for conservative Resize Matrix: "
	    << (float)t / CLOCKS_PER_SEC << std::endl;
  t = clock();
#endif

  uint dataID = datasize-1;

  for(uint i = 0; i < datasize-1; ++i){
    double sv;
    if(!strcmp(_simtype.c_str(), "dot"))
      sv = (SpData[i].cwiseProduct(SpData[dataID])).sum() / (SpData[i].norm() * SpData[i].norm());
    else
      sv = exp(-(SpData[i] - SpData[dataID]).norm() / 2.0);

    if ( sv > _sigma){
      simMat.insert(i, dataID) = sv;
      //_sigmaGraph.insert(dataID, i) = 1;
      L.push_back(i);
    }  
  }

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
  update_Sp_totedge_graph();

#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for init_center_star list and update edge graph:"
	    << (float)t / CLOCKS_PER_SEC << std::endl;
#endif
}
