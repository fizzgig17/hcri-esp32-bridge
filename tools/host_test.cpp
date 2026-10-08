// PC-side check of src/metrics.h: g++ -O2 -I src -o host_test tools/host_test.cpp
// Input lines: name apply_correction(0/1) start_nm n v0 v1 ...  Output: x y CCT Duv Ra R9
#include <stdio.h>
#include <string.h>
#include <vector>
#include <chrono>
#include "metrics.h"
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"r"); char name[64]; int corr,start,n;
  while(fscanf(f,"%63s %d %d %d",name,&corr,&start,&n)==4){
    std::vector<float> v(n); for(int i=0;i<n;i++){double d;fscanf(f,"%lf",&d);v[i]=(float)d;}
    if(corr) apply_correction(v.data(),n,start);
    Metrics m; auto t0=std::chrono::steady_clock::now();
    compute_metrics(v.data(),n,start,&m);
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
    printf("%s %.5f %.5f %.2f %.6f %d %d  (%.1f ms)\n",name,m.x,m.y,m.cct,m.duv,m.ra,m.r9,ms);
  }
}
