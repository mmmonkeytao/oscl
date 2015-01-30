#ifndef ONLINESTARCLUSTERING_H
#define ONLINESTARCLUSTERING_H

#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <algorithm>
#include <fstream>
#include <stdio.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <stdexcept>

#include "Vertex.h"

using namespace Eigen;

namespace oscl {

  typedef SparseVector<double, ColMajor, int> SparseVectorXd;
  typedef std::vector<SparseVectorXd> SparseDataSet;
  
  class OnlineStarClustering  
  {
  public:

    typedef std::vector<Eigen::VectorXd> DataSet;

    OnlineStarClustering(uint feaSize, std::string similaritytype,double sigma = 0.7);
    
    // Destructor
    virtual ~OnlineStarClustering();    
    void clear();

    // access functions
    const std::map<uint, int> & getLabelList(){ return _labels;}
    uint getDataSize();
    double getSigma(){ return _sigma;};
    const std::map<uint, Vertex> getGraph(){ return _graph; };
    const DataSet & getData(){ return _data; };
    const Eigen::MatrixXd& getSimilarityMat() {return _similarityMatrix;};

    // I/O
    void exportClusterInfo(const char* save_dir, uint threshold);
    void exportDot(char const *filename, bool use_data) const;
    void loadAndAddData(char const *filename);
    void loadAndAddSparseData(char const *filename);
    void addPoint(Eigen::VectorXd const &p, uint idx);

    void deleteData(std::list<uint> deleteList);
    void insert(VectorXd &sv);
    void insert(VectorXd v, int label);
    void insertSparseData();    

    // cluster measurement
    void V_measure(double beta, bool message);
    double GaussianKernel(uint id1, uint id2, double sigma2 = 0.2) const;
    void exportSimilarityMat(const char*, bool sort);
 
  protected:

    virtual double computeSimilarity(uint id1, uint id2) const;

    //private:

    typedef Eigen::MatrixXd MatrixType;

    uint _feaSize;
    std::string _simtype;
    double _sigma;
    
    DataSet _data;
    SparseDataSet _spData;
    MatrixType _similarityMatrix;
    // store in sparse way
    std::vector< std::vector<double> > _similarityMat;
        
    std::map<uint, Vertex> _graph; // Thresholded graph.
    std::map<uint, uint>  _id2idx;  // maps from vertex ID to index
    std::list <uint> _elemInGraphSigma;
    Eigen::SparseMatrix<uint> _sigmaGraph;
    std::map<uint, int> _labels;

    unsigned _numClusters = 0;
    double h = 0.0, c = 0.0;
    
    // private functions
    std::priority_queue<Vertex> _priorityQ;

    void insertData(uint start_idx);
    void insertLastPoint(uint new_vertex_id);

    void fastInsert(uint alphaID, std::list <uint> &L);
    void fastUpdate(uint alphaID);
    Eigen::VectorXd readVector(std::ifstream &ifile) const;
    void updateSimilarityMatrix(uint start_idx);
    uint vertexIDMaxDeg(std::list<uint> const &L) const;

    void fastDelete(uint alphaID);
    void sortList(std::list <uint>& AdjCV);

  };
  
}

#endif
