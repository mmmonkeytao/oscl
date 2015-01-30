#include "hmp.h"

void oscl::HMP::max_pooling_layer2(Eigen::VectorXd& rgb_fea, Eigen::VectorXd& gray_fea)
{
  uint fea_x = L2sz[0];
  uint fea_y = L2sz[1];

  uint rgb_feasz = Gamma3C.rows();
  uint gray_feasz = Gamma1C.rows();

  uint pgrid[3] = {pooling[0]*pooling[0], pooling[1]*pooling[1], pooling[2]*pooling[2]};
  pgrid[2] = pgrid[0] + pgrid[1];
  pgrid[1] = pgrid[0];
  pgrid[0] = 0;
  
  for(uint i = 0; i < 3; ++i){

    uint ngrid = pooling[i] * pooling[i];

    // get index belong to ith grid
    double xlength = (double)L2sz[0] / (double)pooling[i];
    double ylength = (double)L2sz[1] / (double)pooling[i];

    std::vector<std::vector<uint> > idx;
    for(uint it = 0; it < ngrid; ++it){
      idx.push_back(std::vector<uint>());
    }
      
    for(uint id_y = 0; id_y < fea_y; ++id_y)
      for(uint id_x = 0; id_x < fea_x; ++id_x){

	uint ith_x = ceil((double)(id_x + 1)/xlength) - 1;
	uint ith_y = ceil((double)(id_y + 1)/ylength) - 1;
	uint ith_grid = ith_y + pooling[i]*ith_x;

	uint idx_f = id_y * fea_x + id_x;
	idx[ith_grid].push_back(idx_f);
      }
      
    for(uint j = 0; j < ngrid; ++j){
      uint size = idx[j].size();
      Eigen::MatrixXd rgbtmp = {rgb_feasz, size};
      Eigen::MatrixXd graytmp = {gray_feasz, size};

      for(uint icol = 0; icol < size; ++icol){
	rgbtmp.col(icol) = Gamma3C.col(idx[j][icol]);
	graytmp.col(icol) = Gamma1C.col(idx[j][icol]);
      }

      rgb_fea.segment((j + pgrid[i])*rgb_feasz, rgb_feasz)
	= rgbtmp.rowwise().maxCoeff();
      gray_fea.segment((j + pgrid[i])*gray_feasz, gray_feasz)
	= graytmp.rowwise().maxCoeff();
    }

  }

  
}

void oscl::HMP::max_pooling_layer1(Eigen::MatrixXd &pooling_l1,
					  const char* type)
{
  uint pooling_row = floor((float)L1sz[0]/(float)encode_first_pooling);
  uint pooling_col = floor((float)L1sz[1]/(float)encode_first_pooling);

  Eigen::MatrixXd *GammaMat = nullptr;
  if(!strcmp(type, "1C")){
    GammaMat = &Gamma1C;
  } else {
    GammaMat = &Gamma3C;
  }

  uint fsz = (*GammaMat).rows();
  
  pooling_l1 = Eigen::MatrixXd::Zero(fsz, pooling_row*pooling_col);

  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(fsz, encode_first_pooling*encode_first_pooling);

  uint col = 0;
  for(uint j = 0; j < pooling_col; ++j)
    for(uint i = 0; i < pooling_row; ++i){

      uint x = i * encode_first_pooling;
      uint y = j * encode_first_pooling;

      uint pcol = 0;
      for(uint l = 0; l < encode_first_pooling; ++l)
	for(uint m = 0; m < encode_first_pooling; ++m){

	  uint idx = (x + m) + L1sz[0] * (y + l);
	  tmp.col(pcol) = (*GammaMat).col(idx);
	  ++pcol;
	}

      pooling_l1.col(col) = tmp.rowwise().maxCoeff();
      ++col;
    }
}


