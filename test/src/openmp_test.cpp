#include "omp.h"
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace std::chrono;

int main()
{
  VectorXd v1{30000}, v2{30000};
  v1.setRandom(); v2.setRandom();
  
// #pragma omp for nowait
//   for(uint i = 0; i < 3*10e10; ++i)
//     double a = 1+1.0;

  //time = omp_get_wtime() - time;
//   cout << time << "\n";

//   #pragma omp parallel
//   printf("Number of threads: %d\n", omp_get_num_threads());

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  
#pragma omp parallel 
  {
    //double time = omp_get_wtime();
#pragma omp for schedule(static)
    for(uint i = 0; i < 5000000; ++i){
      double var = v1.transpose() * v2;
    }
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  cout << "time: " << duration_cast<milliseconds>(t2-t1).count() << endl;
  
 return 0;
}
