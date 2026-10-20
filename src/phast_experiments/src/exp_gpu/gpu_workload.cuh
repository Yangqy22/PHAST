#ifndef PHAST_GPU_WORKLOAD_CUH
#define PHAST_GPU_WORKLOAD_CUH

#include <stdio.h>
extern "C" float *matAdd(float *a,float *b,int length);
extern "C" void kernelCpp(int* h_a, int N, int threadsPerBlock, int blocksPerGrid, int* d_a, cudaStream_t stream1);
#endif
