#include "OnlineStarClustering.h"

using namespace std;
using namespace oscl;

OnlineStarClustering::OnlineStarClustering(uint feaSize, string similaritype, double sigma, double param): _feaSize(feaSize), _simtype(similaritype), _sigma(sigma), _param(param)
{}


OnlineStarClustering::~OnlineStarClustering()
{
  // clear allocated data

}

void OnlineStarClustering::clear()
{
  _data.clear();
  _similarityMatrix.resize(0,0);

  _graph.clear();
  _id2idx.clear();
  _elemInGraphSigma.clear();
}

uint OnlineStarClustering::getDataSize()
{
  return _data.size();
}

double OnlineStarClustering::GaussianKernel(uint id1, uint id2) const
{
  return exp(-(_data[id1].normalized() - _data[id2].normalized()).squaredNorm()/(2*_param));
  //return exp(-(_data[id1] - _data[id2]).squaredNorm()/(2*_param));

}
