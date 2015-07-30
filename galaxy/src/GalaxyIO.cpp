#include "Galaxy.h"

void oscl::Galaxy::exportClustDot(char const *filename) const
{
  std::ofstream ofile(filename);

  ofile << "digraph {" << std::endl;
  ofile << "overlap = false;" << std::endl; 
  ofile << "splines = true;" << std::endl; 
  ofile << "size = \"20,30\";" << std::endl;
  
  for(std::map<uint, Planet>::const_iterator it = _graph.begin(); 
      it != _graph.end(); ++it){

    Planet v = it->second;
    if(v.getType() == Planet::CENTER)
      
      ofile << it->first << "  [shape = doublecircle,style=filled,"
	    << "fontsize = 20,label=\"" << it->first << "\"";
    
    else 
      ofile << it->first << "  [shape = circle,fontsize = 15,label=\""
	    << it->first << "\"";
    
    ofile << "]" << std::endl;
  }

  ofile << "edge [dir=none]" << std::endl;
  for(std::map<uint,Planet>::const_iterator it1 = _graph.begin(); 
      it1 != _graph.end(); ++it1){

    if(it1->second.getType() == Planet::CENTER){
      auto adj_list = it1->second.getConstDomSatsList();

      for(std::set<uint>::const_iterator it2 = adj_list.begin(); 
	  it2 != adj_list.end(); ++it2)
	ofile << it1->first << " -> " << *it2 << ";" << std::endl;
    }
  }

  ofile << "}" << std::endl;
  ofile.close();
}

void oscl::Galaxy::exportClusterInfo(const char* save_dir, uint threshold)
{
  using namespace std;
  // threshold defines cluster with minimumal number to be exported
  ofstream ofile(save_dir);
  uint clust_count = 0;
  
  for(auto it = _graph.begin(); it != _graph.end(); ++it){
    Planet v = it->second;

    if(v.getType() == Planet::CENTER){
      set<uint> ls = v.getDomSatsList();
      ++clust_count;
      
      if(ls.size() >= threshold){

	ofile << "Cluster: " << clust_count << endl;
	ofile << "center's label: " << _labels[v.getID()] << endl;
	ofile << "stars's labels:" << endl;

	for(auto &x: ls){
	  ofile << _labels[x] << " ";
	}
	
	ofile << "\nstars's ID:\n";
	for(auto &x: ls){
	  ofile << x << " ";
	}
	
	ofile << endl;

	uint substars = v.getDegree();
	double attrSum = v.getAttrSum();
	ofile << "Average Similarity: " << attrSum / (double)substars << endl << endl;
	
      }
      
    }
  }

  ofile.close();
  
}

void oscl::Galaxy::saveCenterList() const
{

  std::ofstream ofile("CenterIdx.dat");

  for(auto &x: _centerList)
    ofile << x << std::endl;

  ofile.close();
  
}
