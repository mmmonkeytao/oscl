#include "hmp.h"

// void onlineclust::HMP::depthtonormal(int topleft[2])
// // pcdis is to check
// {
//   /** depth to point cloud */
//   int center[2] = {320, 240};
//   int rows = imgDepth.rows();
//   int cols = imgDepth.cols();
//   double constant = 570.3;

//   Eigen::MatrixXd normal[3];
//   for(uint i = 0; i < 3; ++i){
//     normal[i] = Eigen::MatrixXd{rows, cols};
//     imgNormal[i] = Eigen::MatrixXd::Zero(rows, cols);
//   }
  
//   Eigen::MatrixXd p1 = Eigen::MatrixXd::Ones(rows, cols);
//   Eigen::MatrixXd p2 = Eigen::MatrixXd::Ones(rows, cols);

//   int tmp1 = topleft[0] - 1 - center[0];
//   int tmp2 = topleft[1] - 1 - center[1];

//   for(int c = 0; c < p1.cols(); ++c)
//     p1.col(c) *= static_cast<double>((c+1) + tmp1);    
  
//   for(int r = 0; r < p2.rows(); ++r)
//     p2.row(r) *= static_cast<double>((r+1) + tmp2);

//   normal[0] = p1.array() * imgDepth.array() / (constant * 1000);
//   normal[1] = p2.array() * imgDepth.array() / (constant * 1000);
//   normal[2] = imgDepth / 1000;

//   /** points cloud to normal */
//   // 1 centimeter
//   double threshold = 0.01;
//   // window size
//   int win = 5;
//   uint mindata = 3;
//   uint minh, maxh, minw, maxw;

//   Eigen::EigenSolver<Eigen::MatrixXd> es;
//   for(int i = 0; i < rows; ++i)
//     for(int j = 0; j < cols; ++j){
//       minh = std::max(i - win, 0);
//       maxh = std::min(i + win, rows-1);
//       minw = std::max(j - win, 0);
//       maxw = std::min(j + win, cols-1);

//       //auto pcdis0 = normal[0].block(minh, minw, win1, win1).array() - normal[0](i,j);
//       //auto pcdis1 = normal[1].block(minh, minw, win1, win1).array() - normal[1](i,j);
//       auto patch0 = normal[0].block(minh, minw, maxh-minh+1, maxw-minw+1);
//       auto patch1 = normal[1].block(minh, minw, maxh-minh+1, maxw-minw+1);
//       auto patch2 = normal[2].block(minh, minw, maxh-minh+1, maxw-minw+1);
//       auto pcdis2 = patch2.array() - normal[2](i,j);
//       auto pcdis = pcdis2.abs();
//       double pcij = sqrt(normal[0](i,j)*normal[0](i,j) + normal[1](i,j)*normal[1](i,j) + normal[2](i,j)*normal[2](i,j));

//       Eigen::MatrixXd window = Eigen::MatrixXd::Ones(maxh-minh+1,maxw-minw+1);      
//       auto tmp = pcdis.cwiseMin(pcij*threshold).cwiseNotEqual(window.array()*pcij*threshold);

//       int sum = tmp.count();

//       if( sum > mindata && pcij > 0){
// 	Eigen::MatrixXd subwpc{sum, 3};
// 	uint tmpr = 0;
// 	for(uint l = 0; l < tmp.cols(); ++l)
// 	  for(uint m = 0; m < tmp.rows(); ++m)
// 	    if(tmp(m,l) == 1){
// 	      subwpc(tmpr, 0) = patch0(m,l);
// 	      subwpc(tmpr, 1) = patch1(m,l);
// 	      subwpc(tmpr, 2) = patch2(m,l);
// 	      ++tmpr;
// 	    }
	
// 	auto ones = Eigen::MatrixXd::Ones(sum, 1);
// 	subwpc = subwpc - ones *(subwpc.colwise().sum()/sum);
	
// 	es.compute(subwpc.transpose() * subwpc);
// 	auto vec = es.eigenvectors();
	
//  	imgNormal[0](i,j) = vec(0,0).real();
// 	imgNormal[1](i,j) = vec(1,0).real();
// 	imgNormal[2](i,j) = vec(2,0).real();
	
//       }
     
//     }

//   Eigen::MatrixXd dd{rows, cols};
//   dd.array() = imgNormal[0].array() * normal[0].array();
//   dd.array() += imgNormal[1].array() * normal[1].array();
//   dd.array() += imgNormal[2].array() * normal[2].array();

//   for(uint i = 0; i < dd.rows(); ++i)
//     for(uint j = 0; j < dd.cols(); ++j)
//       if(dd(i,j) > 0.0)
// 	dd(i,j) = 1.0;
//       else if(dd(i,j) < 0.0)
// 	dd(i,j) = -1.0;
//       else
// 	dd(i,j) = 0.0;
  
//   imgNormal[0].array() *= dd.array();
//   imgNormal[1].array() *= dd.array();
//   imgNormal[2].array() *= dd.array();
 
// }


void onlineclust::HMP::depthtonormal(int topleft[2])
// pcdis is to check
{
  /** depth to point cloud */
  int center[2] = {320, 240};
  int rows = img1C.rows;
  int cols = img1C.cols;
  double constant = 570.3;

  cv::Mat normal(rows, cols, CV_64FC3);
  img3C = cv::Mat(rows, cols, CV_64FC3, cv::Scalar(0.0, 0.0, 0.0));

  cv::Mat p1(rows, cols, CV_64FC1, cv::Scalar(1.0));
  cv::Mat p2(rows, cols, CV_64FC1, cv::Scalar(1.0));

  int tmp1 = topleft[0] - 1 - center[0];
  int tmp2 = topleft[1] - 1 - center[1];

  for(int c = 0; c < cols; ++c)
    p1.col(c) *= static_cast<double>((c+1) + tmp1);    
  
  for(int r = 0; r < rows; ++r)
    p2.row(r) *= static_cast<double>((r+1) + tmp2);

  cv::Mat channels[3];
  multiply(p1, img1C, channels[0], 1.0/(constant*1000.0));
  multiply(p2, img1C, channels[1], 1.0/(constant*1000.0));
  channels[2] = img1C / 1000.0;
  merge(channels, 3, normal);
    
  /** points cloud to normal */
  // 1 centimeter
  double threshold = 0.01;
  // window size
  int win = 5;
  uint mindata = 3;
  uint minh, maxh, minw, maxw;

  //Eigen::EigenSolver<Eigen::MatrixXd> es;
  for(int i = 0; i < rows; ++i)
    for(int j = 0; j < cols; ++j){
      minh = std::max(i - win, 0);
      maxh = std::min(i + win, rows-1);
      minw = std::max(j - win, 0);
      maxw = std::min(j + win, cols-1);

      cv::Mat patch0 = channels[0](cv::Range(minh,maxh+1),cv::Range(minw,maxw+1));
      cv::Mat patch1 = channels[1](cv::Range(minh,maxh+1),cv::Range(minw,maxw+1));
      cv::Mat patch2 = channels[2](cv::Range(minh,maxh+1),cv::Range(minw,maxw+1));
      cv::Mat pcdis2 = patch2 - channels[2].at<double>(i,j);
      cv::Mat pcdis = abs(pcdis2);

      cv::Vec<double, 3> v = normal.at<cv::Vec<double, 3> >(i,j);
      double pcij = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

      cv::Mat mask = pcdis < (pcij*threshold);
      cv::Mat smallpcdis(pcdis.rows,pcdis.cols,CV_64FC1,0.0);
      pcdis.copyTo(smallpcdis, mask);

      int sum = countNonZero(mask);
      
      if( sum > mindata && pcij > 0.0){
	
	cv::Mat subwpc(sum, 3, CV_64FC1);
	     
      	uint tmpr = 0;
      	for(uint l = 0; l < smallpcdis.cols; ++l)
      	  for(uint m = 0; m < smallpcdis.rows; ++m)
      	    if(mask.at<uint8_t>(m,l) != 0){
	      
      	      subwpc.at<double>(tmpr, 0) = patch0.at<double>(m,l);
      	      subwpc.at<double>(tmpr, 1) = patch1.at<double>(m,l);
      	      subwpc.at<double>(tmpr, 2) = patch2.at<double>(m,l);
      	      ++tmpr;
      	    }

	cv::Mat colones(sum, 1, CV_64FC1, 1.0);
	cv::Mat rowvec(1, 3, CV_64FC1);
	cv::reduce(subwpc, rowvec, 0, CV_REDUCE_AVG);
	subwpc -= colones * rowvec;

	cv::Mat eigenvectors;
	cv::Mat eigenvalues;
	eigen(subwpc.t() * subwpc, eigenvalues, eigenvectors);

	// find eigenvector with maximal eigenvalue
	uint maxloc = 0;
	double maxv = eigenvalues.at<double>(0);
	if(eigenvalues.at<double>(1) > maxv)  {
	  maxloc = 1; maxv = eigenvalues.at<double>(1);
	}
	if(eigenvalues.at<double>(2) > maxv)
	  maxloc = 2;

	img3C.at<cv::Vec<double, 3> >(i,j) = cv::Vec<double, 3>(eigenvectors.row(maxloc)); 	
      }
     
    }

  cv::Mat dd;
  multiply(img3C, normal, dd);
  cv::Mat ddchs[3];
  split(dd, ddchs);
  cv::Mat ddsum = ddchs[0] + ddchs[1] + ddchs[2];

  cv::Mat mask = ddsum > 0.0;
  ddsum.setTo(1.0, mask);
  mask = ddsum < 0.0;
  ddsum.setTo(-1.0,mask);

  cv::Mat img3Cchs[3];
  split(img3C, img3Cchs);
  multiply(img3Cchs[0], ddsum, img3Cchs[0]);
  multiply(img3Cchs[1], ddsum, img3Cchs[1]);
  multiply(img3Cchs[2], ddsum, img3Cchs[2]);
  merge(img3Cchs, 3, img3C);

}
