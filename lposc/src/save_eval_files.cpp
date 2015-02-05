#include "LPOSC.h"

void oscl::LPOSC::save_eval_files(const char* prefix)
{
  string savepath;
  string strprefix(prefix);
  string iterpath("_qual_per_iter.dat");

  vector<double> lch_threshold, lch_edges, lch_alledges;
  vector<double> role_ch;
  vector<double> Hqueries;
  vector<double> singleNodes;
  vector<double> domcenterch;

  lch_threshold.push_back(1.0);
  lch_edges.push_back(1.0);
  lch_alledges.push_back(1.0);
  role_ch.push_back(1.0);
  Hqueries.push_back(1.0);
  singleNodes.push_back(1.0);
  domcenterch.push_back(1.0);

  uint iter_num = totlabels_edges.size();
  
  for(uint i = 1; i < iter_num; ++i){

    uint current_size = totlabels_edges[i].size();
    uint previous_size = totlabels_edges[i-1].size();
    
    // label changed
    int counter1 = 0, counter2 = 0, counter3 = 0;
    
    for(uint j = 0; j < previous_size; ++j){
      // if(totlabels_threshold[i](j) != totlabels_threshold[i-1](j))
      // 	++counter1;
      
      if(totlabels_edges[i](j) != totlabels_edges[i-1](j))
	++counter2;
      
      // if(totlabels_alledges[i](j) != totlabels_alledges[i-1](j))
      // 	++counter3;
    }

    //lch_threshold.push_back((float)counter1/(float)previous_size);
    lch_edges.push_back((float)counter2/(float)previous_size);
    //lch_alledges.push_back((float)counter3/(float)previous_size);

    // center-star changed
    VectorXi vold = VectorXi::Zero(previous_size), vnew = VectorXi::Zero(previous_size);

    for(auto &x: allcenterslist[previous_size-1])
      vold(x) = 1;
    for(auto &x: allcenterslist[current_size-1])
      if(x < previous_size)
	vnew(x) = 1;
    int NotEqual = (vold.cwiseNotEqual(vnew)).sum();   
    role_ch.push_back((float)NotEqual/(float)previous_size);
    
    // star's dom center changed
    counter1 = 0;
    for(auto &x: allstarslist[previous_size-1])
      if(_graph[x].getType() == Vertex::SATELLITE){
	auto iter = std::find(allstarslist[current_size-1].begin(), allstarslist[current_size-1].end(), x);
	if(iter != allstarslist[current_size-1].end() && _graph[*iter].getType() == Vertex::SATELLITE){
	  if(_graph[x].getDomCenter() != _graph[*iter].getDomCenter()){
	    	    ++counter1;
	  }

	}
      }
    domcenterch.push_back((float)counter1/(float)previous_size);

    // single nodes
    counter1 = 0;
    for(uint j = 0; j < current_size; ++j){
      Vertex vertex = _graph[j];
      if(vertex.getType() == Vertex::CENTER){
	if(_graph[j].getDomSatsList().size() == 0)
	  ++counter1;
      }
    }
    singleNodes.push_back((float)counter1/(float)current_size);
    // Hqueries
    Hqueries.push_back((float)(allcenterslist[current_size-1].size()-counter1)/(float)current_size);
        
  }

  // save per iteration file
  savepath = strprefix + iterpath;
  ofstream of1(savepath.c_str());

  //  of1 << "threshold edges alledges rolechange Hqueries singleNodes Homo Completeness DomCenterCh" << std::endl;
  of1 << "edges rolechange Hqueries singleNodes Homo Completeness DomCenterCh" << std::endl;

  //for(uint i = 0; i < lch_threshold.size(); ++i)
  //  of1 << lch_threshold[i] << " ";
  //of1 << std::endl;
  for(uint i = 0; i < lch_edges.size(); ++i)
    of1 << lch_edges[i] << " ";
  of1 << std::endl;
  //for(uint i = 0; i < lch_alledges.size(); ++i)
  //  of1 << lch_alledges[i] << " ";
  //of1 << std::endl;
  for(uint i = 0; i < role_ch.size(); ++i)
    of1 << role_ch[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < Hqueries.size(); ++i)
    of1 << Hqueries[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < singleNodes.size(); ++i)
    of1 << singleNodes[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < homo.size(); ++i)
    of1 << homo[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < complete.size(); ++i)
    of1 << complete[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < domcenterch.size(); ++i)
    of1 << domcenterch[i] << " ";
  of1 << std::endl;
  of1.close();

  // save label proped at each iteration
  // savepath = strprefix + "_all_labels_threshold.dat";

  // //of1 = ofstream(savepath.c_str());
  // of1.open(savepath.c_str());  
  // for(uint i = 0; i < totlabels_threshold.size(); ++i){
  //   VectorXi v = VectorXi::Zero(datasize);
  //   v.head(totlabels_threshold[i].size()) = totlabels_threshold[i];
  //   of1 << v.transpose() << std::endl;
  // }
  // of1.close();
  
  savepath = strprefix + "_all_labels_cedges.dat";
  //of1 = ofstream(savepath.c_str());
  of1.open(savepath.c_str());
  for(uint i = 0; i < totlabels_edges.size(); ++i){
    VectorXi v = VectorXi::Zero(datasize);
    v.head(totlabels_edges[i].size()) = totlabels_edges[i];
    of1 << v.transpose() << std::endl;
  }
  of1.close();

  // savepath = strprefix + "_all_labels_alledges.dat";
  // //of1 = ofstream(savepath.c_str());
  // of1.open(savepath.c_str());
  // for(uint i = 0; i < totlabels_alledges.size(); ++i){
  //   VectorXi v = VectorXi::Zero(datasize);
  //   v.head(totlabels_alledges[i].size()) = totlabels_alledges[i];
  //   of1 << v.transpose() << std::endl;
  // }
  // of1.close();

}
