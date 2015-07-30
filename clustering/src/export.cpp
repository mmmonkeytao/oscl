#include "OnlineStarClustering.h"

void oscl::OnlineStarClustering::exportDot(char const *filename, bool use_data) const
{
  std::ofstream ofile(filename);

  ofile << "digraph {" << std::endl;
  if(use_data)
    ofile << "overlap = true;" << std::endl; 
  else
    ofile << "overlap = false;" << std::endl; 
  ofile << "splines = true;" << std::endl; 
  ofile << "size = \"20,30\";" << std::endl;
  
  for(std::map<uint,Vertex>::const_iterator it = _graph.begin(); 
      it != _graph.end(); ++it){

    Vertex v = it->second;
    if(v.getType() == Vertex::CENTER)
      
      ofile << it->first << "  [shape = doublecircle,style=filled,"
	    << "fontsize = 20,label=\"" << _labels.at(it->first) << "\"";
    
    else 
      ofile << it->first << "  [shape = circle,fontsize = 20,label=\""
	    << _labels.at(it->first) << "\"";
    
    if(use_data){
      uint point_idx = it->first;//_id2idx.at(it->first);
      ofile << ",pos = \""  << _data[point_idx](0) << "," 
	    << _data[point_idx](1) << "!\"";

    }
    ofile << "]" << std::endl;
  }

  ofile << "edge [dir=none]" << std::endl;
  for(std::map<uint,Vertex>::const_iterator it1 = _graph.begin(); 
      it1 != _graph.end(); ++it1){

    if(it1->second.getType() == Vertex::CENTER){
      auto adj_list = it1->second.getConstDomSatsList();//it1->second.getAdjVerticesList();

      for(std::list<uint>::const_iterator it2 = adj_list.begin(); 
	  it2 != adj_list.end(); ++it2)
	ofile << it1->first << " -> " << *it2 << ";" << std::endl;
    }
  }

  ofile << "}" << std::endl;
  ofile.close();
}

void oscl::OnlineStarClustering::exportClustDot(char const *filename) const
{
  std::ofstream ofile(filename);

  ofile << "digraph {" << std::endl;
  ofile << "overlap = false;" << std::endl; 
  ofile << "splines = true;" << std::endl; 
  ofile << "size = \"20,30\";" << std::endl;
  
  for(std::map<uint,Vertex>::const_iterator it = _graph.begin(); 
      it != _graph.end(); ++it){

    Vertex v = it->second;
    if(v.getType() == Vertex::CENTER)
      
      ofile << it->first << "  [shape = doublecircle,style=filled,"
	    << "fontsize = 20,label=\"" << it->first << "\"";
    
    else 
      ofile << it->first << "  [shape = circle,fontsize = 15,label=\""
	    << it->first << "\"";
    
    ofile << "]" << std::endl;
  }

  ofile << "edge [dir=none]" << std::endl;
  for(std::map<uint,Vertex>::const_iterator it1 = _graph.begin(); 
      it1 != _graph.end(); ++it1){

    if(it1->second.getType() == Vertex::CENTER){
      auto adj_list = it1->second.getConstDomSatsList();

      for(std::list<uint>::const_iterator it2 = adj_list.begin(); 
	  it2 != adj_list.end(); ++it2)
	ofile << it1->first << " -> " << *it2 << ";" << std::endl;
    }
  }

  ofile << "}" << std::endl;
  ofile.close();
}

void oscl::OnlineStarClustering::exportClusterInfo(const char* save_dir, uint threshold)
{
  using namespace std;
  // threshold defines cluster with minimumal number to be exported
  ofstream ofile(save_dir);
  uint clust_count = 0;
  
  for(auto it = _graph.begin(); it != _graph.end(); ++it){
    Vertex v = it->second;

    if(v.getType() == Vertex::CENTER){
      list<uint> ls = v.getDomSatsList();
      ++clust_count;
      
      if(ls.size() >= threshold){
	double similarity = 0.0;
	ofile << "Cluster: " << clust_count << endl;
	ofile << "center's label: " << _labels[v.getID()] << endl;
	ofile << "stars's labels:" << endl;

	VectorXd center = _data[v.getID()];
	for(auto &x: ls){
	  ofile << _labels[x] << " ";
	  similarity += computeSimilarity(v.getID(), x);
	}
	ofile << "\nstars's ID:\n";
	for(auto &x: ls){
	  ofile << x << " ";
	}
	
	ofile << endl;
	ofile << "Average Similarity: " << similarity/ls.size() << endl << endl;
	
      }
      
    }
  }

  ofile.close();
  
}

void oscl::OnlineStarClustering::exportSimilarityMat(const char* save_dir, bool sort)
{
  using namespace Eigen;
  using namespace std;
  
  uint dataSize = _data.size();

  if(dataSize == 0){
    cerr << "Data size is null, nothing to export\n";
    return;
  }
     
  if(sort){
    // sort horizontally
    for(uint line = 0; line < dataSize; ++line){

      for(uint i = 0; i < dataSize; ++i)
	for(uint j = i+1; j < dataSize; ++j){
	  if(_similarityMatrix(line, i) > _similarityMatrix(line, j)){
	    double temp = _similarityMatrix(line, i);
	    _similarityMatrix(line, i) = _similarityMatrix(line, j);
	    _similarityMatrix(line, j) = temp;
	  }
	  
	}
      
    }

    // sort vertically
    for(uint i = 0; i < dataSize; ++i)
      for(uint j = i+1; j < dataSize; ++j){
	if(_similarityMatrix.row(i).norm() < _similarityMatrix.row(j).norm()){
	  RowVectorXd temp;
	  temp = _similarityMatrix.row(i); 
	  _similarityMatrix.row(i) = _similarityMatrix.row(j);
	  _similarityMatrix.row(j) = temp;
	}
      }
  }

  ofstream ofile(save_dir);

  ofile << dataSize << endl;

  ofile << _similarityMatrix << endl;
  
  ofile.close();
  
}
