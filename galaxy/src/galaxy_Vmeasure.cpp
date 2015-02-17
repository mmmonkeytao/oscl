#include "Galaxy.h"

// print out V_measure result
void oscl::Galaxy::V_measure(double beta, bool message){

  using namespace std;
  
  map<int, list<uint> > C_list;  // Class list
  list<uint> K_list;  // Cluster list

  uint data_size = _datasize; 

  // obtain ground truth
  for(auto it1 = _graph.begin(); it1 != _graph.end(); ++it1){

    if((it1->second).getType() == Planet::CENTER){
      K_list.push_back(it1->first);
    }

    int label = _labels.at(it1->first);
    if( C_list.find(label) != C_list.end() ){

      C_list.at(label).push_back( it1->first );

    } else {
      list<uint> ls;
      ls.push_back(it1->first);
      C_list.insert(pair<int, list<uint> >(label, ls));
    }
  }

  double H_CK = 0.0, H_C = 0.0;
  // homogeneity
  for(auto it = K_list.begin(); it != K_list.end(); ++it){

    auto ls = _graph[*it].getDomSatsList();
    uint K_size = ls.size() + 1;
    
    for(auto it1 = C_list.begin(); it1 != C_list.end(); ++it1){

      uint a_CK = 0;
      
      for(auto it2 = ls.begin(); it2 != ls.end(); ++it2 ){
	if(it1->first == _labels[*it2]) a_CK++;
      }
      
      if(it1->first == _labels[*it]) a_CK++;

      if(a_CK != 0){
	H_CK += (double)a_CK/data_size * log10( (double)a_CK/K_size );
      }

    }
  }

  H_CK = -H_CK;
  
  for(auto it = C_list.begin(); it != C_list.end(); ++it)
      H_C += (double)(it->second).size()/data_size
	                    * log10((double)(it->second).size()/data_size); 
  
  H_C = -H_C;

  if(H_CK == 0.0)
    _h = 1.0;
  else 
    _h = 1.0 - H_CK / H_C;

  // completeness
  double H_KC = 0.0, H_K = 0.0;
  for(auto it = C_list.begin(); it != C_list.end(); ++it){

    for(auto it1 = K_list.begin(); it1 != K_list.end(); ++it1){
      auto ls = _graph[*it1].getDomSatsList();

      uint a_CK = 0;

      if(it->first == _labels[*it1]) a_CK++;
	
      for(auto it2 = ls.begin(); it2 != ls.end(); ++it2){
	if(_labels[*it2] == it->first) a_CK++;
      }

      if(a_CK != 0){
	H_KC += (double)a_CK/data_size * log10((double)a_CK/(it->second).size());
      }

    }
  }

  H_KC = -H_KC;

  for(auto it = K_list.begin(); it != K_list.end(); ++it){
    uint a_CK = _graph[*it].getDomSatsList().size() + 1;
    if(a_CK != 0) H_K += (double)a_CK/data_size * log10( (double)a_CK/data_size);
  }

  H_K = -H_K;

  if(H_KC == 0.0)
    _c = 1.0;
  else 
    _c = 1.0 - H_KC / H_K;

  if(message){
    cout << "Homogeneity is: " << _h << endl;
    cout << "Completeness is: " << _c << endl;
    cout << "V_measure is: " << (1+beta)*_h*_c/((beta*_h)+_c) << endl; 
    cout << "Number of Clusters: "<< K_list.size() << endl;
  }
}
