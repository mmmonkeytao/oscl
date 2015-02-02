#include "LPOSC.h"

oscl::LPOSC::LPOSC(uint feaSize, string feaType, double sigma) 
  :OnlineStarClustering(feaSize, feaType, sigma)
{}

void oscl::LPOSC::calc_Vmeasure()
{
  V_measure(1, false);
  homo.push_back(h);
  complete.push_back(c);  
}

void oscl::LPOSC::update_totedge_graph()
{
  totedges_graph.conservativeResize(datasize, datasize);
  totedges_graph.bottomRows(1).setZero();
  totedges_graph.rightCols(1).setZero();
  
  for(auto &id1: allcenterslist[datasize-1]){
    auto ls = _graph[id1].getDomSatsList();

    for(auto &id2: ls){
      totedges_graph(id2, id1) = totedges_graph(id1, id2) = 1;
    }    
  }  
}

void oscl::LPOSC::update_Sp_totedge_graph()
{
  Sp_totedges_graph.conservativeResize(datasize, datasize);
  
  for(auto &id1: allcenterslist[datasize-1]){
    auto ls = _graph[id1].getDomSatsList();

    for(auto &id2: ls){
      //totedges_graph(id2, id1) = totedges_graph(id1, id2) = 1;
      if(Sp_totedges_graph.coeff(id2,id1) == 0){
	Sp_totedges_graph.insert(id2,id1) = 1;
	Sp_totedges_graph.insert(id1,id2) = 1;
      }	
    }    
  }  
}

void oscl::LPOSC::update_current_edge_graph()
{
  current_edges_graph = Eigen::MatrixXd::Zero(datasize, datasize);

  for(auto &id1: allcenterslist[datasize-1]){
    auto ls = _graph[id1].getDomSatsList();

    for(auto &id2: ls){
     current_edges_graph(id2, id1) = current_edges_graph(id1, id2) = 1;
    }
  }  
}

void oscl::LPOSC::init_center_star_list()
{

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

