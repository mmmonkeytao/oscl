#include "LPOSC.h"

oscl::LPOSC::LPOSC(uint feaSize, string feaType, double sigma) 
  :OnlineStarClustering(feaSize, feaType, sigma)
{}

void oscl::LPOSC::center_label_propagation(const char* type)
{
  uint datasize = _data.size();
  
  if(!strcmp(type, "threshold")){

    current_threshold_graph = MatrixXd{datasize, datasize};
    current_threshold_graph = MatrixXd(_sigmaGraph).array() * _similarityMatrix.array();// + MatrixXd::Identity(datasize, datasize).array();
    
    jacobi_label_propagation();
    totlabels_threshold.push_back(Ylabel);

  } else if(!strcmp(type, "current_edge")){

    current_threshold_graph = MatrixXd{datasize, datasize};
    current_threshold_graph = MatrixXd(_sigmaGraph).array() * totedges_graph.array();// + MatrixXd::Identity(datasize, datasize).array();

    jacobi_label_propagation();    
    totlabels_edges.push_back(Ylabel);    

  } else if(!strcmp(type, "all_edge")){

    current_threshold_graph = MatrixXd{datasize, datasize};
    current_threshold_graph = MatrixXd(_sigmaGraph).array() * current_edges_graph.array();// + MatrixXd::Identity(datasize, datasize).array();

    jacobi_label_propagation();    
    totlabels_alledges.push_back(Ylabel);
  } else {
    std::cerr << "Unknown label propagation type!\n";
  }
  
}

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

  uint iter_num = totlabels_threshold.size();
  
  for(uint i = 1; i < iter_num; ++i){

    // label changed
    int counter1 = 0, counter2 = 0, counter3 = 0;
    
    for(uint j = 0; j < totlabels_threshold[i-1].size(); ++j){
      if(totlabels_threshold[i](j) != totlabels_threshold[i-1](j))
	++counter1;
      
      if(totlabels_edges[i](j) != totlabels_edges[i-1](j))
	++counter2;
      
      if(totlabels_alledges[i](j) != totlabels_alledges[i-1](j))
	++counter3;
    }

    lch_threshold.push_back((float)counter1/(float)(totlabels_threshold[i-1].size()));
    lch_edges.push_back((float)counter2/(float)(totlabels_threshold[i-1].size()));
    lch_alledges.push_back((float)counter3/(float)(totlabels_threshold[i-1].size()));

    // center-star changed
    VectorXi vold = VectorXi::Zero(totlabels_threshold[i-1].size()), vnew = VectorXi::Zero(totlabels_threshold[i-1].size());
    for(auto &x: allcenterslist[totlabels_threshold[i-1].size()-1])
      vold(x) = 1;
    for(auto &x: allcenterslist[totlabels_threshold[i].size()])
      if(x < allcenterslist[totlabels_threshold[i-1].size()-1].size())
	vnew(x) = 1;
    // star's dom center changed
    counter1 = 0;
    for(auto &x: allstarslist[totlabels_threshold[i-1].size()-1])
      if(_graph[x].getType() == Vertex::SATELLITE){
	auto iter = std::find(allstarslist[totlabels_threshold[i].size()-1].begin(), allstarslist[totlabels_threshold[i].size()-1].end(), x);
	if(iter != allstarslist[totlabels_threshold[i].size()-1].end() && _graph[*iter].getType() == Vertex::SATELLITE){
	  if(_graph[x].getDomCenter() != _graph[*iter].getDomCenter())
	    ++counter1;
	}
      }
    domcenterch.push_back(counter1);

    int NotEqual = (vold.cwiseNotEqual(vnew)).sum();   
    role_ch.push_back((float)NotEqual/(float)totlabels_threshold[i-1].size());

    // single nodes
    counter1 = 0;
    for(uint j = 0; j < totlabels_threshold[i].size(); ++j){
      Vertex vertex = _graph[j];
      if(vertex.getType() == Vertex::CENTER){
	if(_graph[j].getDomSatsList().size() == 0)
	  ++counter1;
      }
    }
    singleNodes.push_back((float)counter1/(float)totlabels_threshold[i].size());
    // Hqueries
    Hqueries.push_back((float)(allcenterslist[totlabels_threshold[i].size()-1].size()-counter1)/(float)totlabels_threshold[i].size());
        
  }

  // save per iteration file
  savepath = strprefix + iterpath;
  ofstream of1(savepath.c_str());

  of1 << "threshold edges alledges rolechange Hqueries singleNodes Homo Completeness DomCenterCh" << std::endl;

  for(uint i = 0; i < lch_threshold.size(); ++i)
    of1 << lch_threshold[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < lch_edges.size(); ++i)
    of1 << lch_edges[i] << " ";
  of1 << std::endl;
  for(uint i = 0; i < lch_alledges.size(); ++i)
    of1 << lch_alledges[i] << " ";
  of1 << std::endl;
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
  savepath = strprefix + "_all_labels_threshold.dat";
  of1 = ofstream(savepath.c_str());
  for(uint i = 0; i < totlabels_threshold.size(); ++i){
    VectorXi v = VectorXi::Zero(_data.size());
    v.head(totlabels_threshold[i].size()) = totlabels_threshold[i];
    of1 << v.transpose() << std::endl;
  }
  of1.close();
  
  savepath = strprefix + "_all_labels_cedges.dat";
  of1 = ofstream(savepath.c_str());
  for(uint i = 0; i < totlabels_edges.size(); ++i){
    VectorXi v = VectorXi::Zero(_data.size());
    v.head(totlabels_edges[i].size()) = totlabels_edges[i];
    of1 << v.transpose() << std::endl;
  }
  of1.close();

  savepath = strprefix + "_all_labels_alledges.dat";
  of1 = ofstream(savepath.c_str());
  for(uint i = 0; i < totlabels_alledges.size(); ++i){
    VectorXi v = VectorXi::Zero(_data.size());
    v.head(totlabels_alledges[i].size()) = totlabels_alledges[i];
    of1 << v.transpose() << std::endl;
  }
  of1.close();

}


void oscl::LPOSC::jacobi_label_propagation()
{
  uint label_num = labelist.size();
  uint datasize = _data.size();

  Eigen::MatrixXd selectgraph{datasize, label_num};

  uint i = 0;
  int loclabels[labelist.size()];

  initialize_Ainv();

  for(auto &x: labelist){
    
    loclabels[i] = x.first;

    initialize_current_label_vector(x.first);

    Eigen::VectorXd Yt, Yt1 = Y0;

    do{
      Yt = Yt1;    
      Yt1 = Ainv * (u * current_threshold_graph * Yt + Y0);
    }while( (Yt - Yt1).norm() > 10e-6 );

    selectgraph.col(i) = Yt1;        
    ++i;
  }

  Ylabel = VectorXi{datasize};

  for(uint i = 0; i < datasize; ++i){
    RowVectorXd::Index maxrow;
    selectgraph.row(i).maxCoeff(&maxrow);

    Ylabel(i) = loclabels[maxrow];
  }

}

void oscl::LPOSC::initialize_Ainv()
{
  Ainv = Eigen::MatrixXd::Identity(_data.size(), _data.size());

  Eigen::VectorXd W(_data.size());
  W = current_threshold_graph.rowwise().sum();

  W = (W*u).array() + u*eps;

  for(auto &x: allcenterslist[_data.size()-1]){
    W(x) += 1; 
  }

  W = W.array().inverse();    
  Ainv = MatrixXd(W.asDiagonal());    
}

void oscl::LPOSC::initialize_current_label_vector(int label)
{
  Y0 = Eigen::VectorXd::Zero(_data.size()); 

  for(auto &x: allcenterslist[_data.size()-1])
    if(_labels[x] == label)
      Y0(x) = 1.0;
    else
      Y0(x) = -1.0;

}

void oscl::LPOSC::insert(VectorXd vec, int label)
{
  if(vec.size() != _feaSize)
    throw std::runtime_error("\nFeature size error!\n");

  _data.push_back(vec);
  // label = -1 if no label
  _labels[_data.size()-1] = static_cast<int>(label);

  // insert into label list
  if(labelist.find(label) != labelist.end()){
    labelist[label]++;
  }else{
    labelist[label] = 1;
  }

  // adjacent list
  std::list<uint> L;
    
  // update similarity matrix and sigma graph
  _sigmaGraph.conservativeResize(_data.size(), _data.size());
  _similarityMatrix.conservativeResize(_data.size(), _data.size());
  
  uint dataID = _data.size()-1;
  for(uint i = 0; i < _data.size()-1; ++i){

    double sv = computeSimilarity(i, dataID);
    _similarityMatrix(i, dataID) = _similarityMatrix(dataID, i) = sv;

    if ( sv > _sigma){        
      _sigmaGraph.insert(dataID, i) = 1;
      L.push_back(i);
    }

  }

  fastInsert(dataID, L);
  L.clear();

  //init_center_star_list();
  //update_edge_graph();

}

void oscl::LPOSC::calc_Vmeasure()
{
  V_measure(1, false);
  homo.push_back(h);
  complete.push_back(c);  
}

void oscl::LPOSC::update_edge_graph()
{
  totedges_graph.conservativeResize(_data.size(), _data.size());
  totedges_graph.bottomRows(1).setZero();
  totedges_graph.rightCols(1).setZero();
  
  current_edges_graph = Eigen::MatrixXd::Zero(_data.size(), _data.size());
  for(auto &id1: allcenterslist[_data.size()-1]){
    auto ls = _graph[id1].getDomSatsList();

    for(auto &id2: ls){
      totedges_graph(id2, id1) = totedges_graph(id1, id2) = 1;
      current_edges_graph(id2, id1) = current_edges_graph(id1, id2) = 1;
    }
    
  }
  
}

void oscl::LPOSC::init_center_star_list()
{
  uint datasize = _data.size();
  vector<uint> centerslist;
  vector<uint> starslist;
  
  for(uint i = 0; i < datasize; ++i)
    if(_graph[i].getType() == Vertex::CENTER){
      centerslist.push_back(i);
    } else {
      starslist.push_back(i);
    }

  allcenterslist.push_back(centerslist);
  allstarslist.push_back(starslist);
}

