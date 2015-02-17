#ifndef GALAXY_H
#define GALAXY_H

#include <iostream>
#include <map>
#include <vector>
#include <list>
#include <set>

#include <cmath>
#include <string>
#include <algorithm>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <stxxl/vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <stdexcept>

#include "Planet.h"

using namespace Eigen;

namespace oscl {

  class Galaxy
  {
  public:

    typedef std::vector<VectorXd> DataSet;

    // used for large dataset
    typedef stxxl::VECTOR_GENERATOR<VectorXd>::result DataSetXXL;
    
    // constructor
    Galaxy(uint feaSize, const char* file);
    Galaxy(uint feaSize, std::string similarityType);
    Galaxy(uint feaSize, std::string similarityType, double param);
    Galaxy(uint feaSize, std::string similarityType, double param, const char* file);
    
    // Destructor
    virtual ~Galaxy();    

    // access functions
    uint getDataSize();
    void incrementDataSize();
    const std::vector<uint>& getCenterCheckNum();
    const std::vector<uint>& getStarBrokenNum();
    
    const DataSet& getData();
    const std::map<uint, int>& getLabelList();
    const std::map<uint, Planet> getGraph();
    const Eigen::MatrixXd& getSimilarityMat();

    // I/O
    void exportClusterInfo(const char* save_dir, uint threshold);
    void exportClustDot(char const*filename) const;

    // insert data
    void unloaded_insert(uint id, int label, uint iter, double select_threshold);

    void loaded_insert_noSim(Eigen::VectorXd vec, uint id, int label, uint iter, double select_threshold);
    
    // cluster evaluation
    void V_measure(double beta, bool message);
    void Hquery_accuracy();
    
    // load fea pool
    void set_fea_pool_null();
    void load_fea_pool(uint id);
    void load_fea_pool(std::set<uint>&);
    void load_fea_pool(std::map<double, uint, std::greater<double> >&, std::map<double, uint, std::greater<double> >::iterator&);

  protected:

    virtual double unloaded_computeSimilarity(uint id1, uint id2) const;
    virtual double loaded_computeSimilarity(uint id1, uint id2);
    virtual double computeSimilarity(uint id1, uint id2);
    
  private:
    typedef Eigen::MatrixXd MatrixType;

    uint _feaSize;
    std::string _simtype;
    double _param;
    const std::string _fea_path_prefix;
    std::string _path_prefix;
    std::map<uint, Eigen::VectorXd> _current_fea_pool;
    
    DataSet _data;
    DataSet _dataXXL;
    
    int _datasize;
    std::map<uint, uint> _dataIDs; // a map from iter to id
    MatrixType _similarityMatrix;
    std::set<uint> _centerList;
        
    std::map<uint, Planet> _graph;     
    std::map<uint, int> _labels;
    std::map<int, uint> _labelist;

    uint _numClusters;
    double _h, _c;

    const double _min_cluster_size_wanted = 25.0;
    const double _clust_size_eps = 0.0002;
    const double _threshold_eps = 0.0002;
    
    // measures for cluster evaluation
    std::vector<uint> center_check_num;
    std::vector<uint> star_broken_num;
  };
  
}

#endif
