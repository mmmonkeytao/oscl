#include "LPOSC.h"

void oscl::LPOSC::center_label_propagation(const char* type)
{

  current_threshold_graph = MatrixXd{datasize, datasize};

  if(!strcmp(type, "threshold")){

    current_threshold_graph = _similarityMatrix;
    jacobi_label_propagation();
    totlabels_threshold.push_back(Ylabel);

  } else if(!strcmp(type, "current_edge")){
    
    update_current_edge_graph();
    current_threshold_graph = _similarityMatrix.array() * current_edges_graph.array();

    jacobi_label_propagation();    
    totlabels_edges.push_back(Ylabel);    

  } else if(!strcmp(type, "all_edge")){
    
    current_threshold_graph = _similarityMatrix.array() * totedges_graph.array();

    jacobi_label_propagation();    
    totlabels_alledges.push_back(Ylabel);
  } else {
    std::cerr << "Unknown label propagation type!\n";
  }
  
}

void oscl::LPOSC::Sp_center_label_propagation(const char* type)
{

  current_threshold_graph = MatrixXd{datasize, datasize};

  if(!strcmp(type, "threshold")){

    current_threshold_graph = SpMatd(simMat.transpose()) + simMat;
    jacobi_label_propagation();
    totlabels_threshold.push_back(Ylabel);

  } else if(!strcmp(type, "current_edge")){
    
    update_current_edge_graph();
    current_threshold_graph =  (SpMatd(simMat.transpose()) + simMat).cwiseProduct(current_edges_graph);
							    
    jacobi_label_propagation();    
    totlabels_edges.push_back(Ylabel);    

  } else if(!strcmp(type, "all_edge")){

    current_threshold_graph = (SpMatd(simMat.transpose()) + simMat).cwiseProduct(Eigen::MatrixXd(Sp_totedges_graph));

    jacobi_label_propagation();    
    totlabels_alledges.push_back(Ylabel);
  } else {
    std::cerr << "Unknown label propagation type!\n";
  }
  
}

void oscl::LPOSC::jacobi_label_propagation()
{
  uint label_num = labelist.size();

  Eigen::MatrixXd selectgraph{datasize, label_num};

  uint i = 0;
  int loclabels[labelist.size()];

#ifdef OPTIMIZE
  clock_t t = clock();
#endif  
  
  initialize_Ainv();

  for(auto &x: labelist){
    
    loclabels[i] = x.first;

    initialize_current_label_vector(x.first);

    Eigen::VectorXd Yt, Yt1 = Y0;

    do{
      Yt = Yt1;    
      Yt1 = Ainv.array() * (u * current_threshold_graph * Yt + Y0).array();
      
    }while( (Yt - Yt1).norm() > 10e-3 );

    selectgraph.col(i) = Yt1;        
    ++i;
  }
  
  Ylabel = VectorXi{datasize};

  for(uint i = 0; i < datasize; ++i){
    RowVectorXd::Index maxrow;
    selectgraph.row(i).maxCoeff(&maxrow);

    Ylabel(i) = loclabels[maxrow];
  }
#ifdef OPTIMIZE
  t = clock() - t;
  std::cout << "Time spent for Jacobi label propagation: " << (float)t/CLOCKS_PER_SEC<< std::endl;
#endif
  
}

void oscl::LPOSC::initialize_Ainv()
{
  Ainv = Eigen::VectorXd::Zero(datasize);
  Ainv = current_threshold_graph.rowwise().sum();

  Ainv = (Ainv*u).array() + u*eps;

  for(auto &x: allcenterslist[datasize-1]){
    Ainv(x) += 1; 
  }

  Ainv = Ainv.array().inverse();    
}

void oscl::LPOSC::initialize_current_label_vector(int label)
{
  Y0 = Eigen::VectorXd::Zero(datasize); 

  for(auto &x: allcenterslist[datasize-1])
    if(_labels[x] == label)
      Y0(x) = 1.0;
    else
      Y0(x) = -1.0;

}
