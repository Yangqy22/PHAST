#include "gpu_workload.cuh"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

template<typename T>
__global__ void matAdd_cuda(T *a,T *b,T *sum)
{
	int i = blockIdx.x*blockDim.x+ threadIdx.x;
    sum[i] = a[i] + b[i];
}

float* matAdd(float *a,float *b,int length)
{
    int device = 5;
    cudaSetDevice(device);
    cudaDeviceProp devProp;
    cudaGetDeviceProperties(&devProp, device);
    int threadMaxSize = devProp.maxThreadsPerBlock;
    int blockSize = (length+threadMaxSize-1)/threadMaxSize;
    dim3 thread(threadMaxSize);
    dim3 block(blockSize);
    int size = length * sizeof(float);
    float *sum =(float *)malloc(size);
    float *sumGPU,*aGPU,*bGPU;
    cudaMalloc((void**)&sumGPU,size);
    cudaMalloc((void**)&aGPU,size);
    cudaMalloc((void**)&bGPU,size);
    cudaMemcpy((void*)aGPU,(void*)a,size,cudaMemcpyHostToDevice);
    cudaMemcpy((void*)bGPU,(void*)b,size,cudaMemcpyHostToDevice);
    matAdd_cuda<float><<<block,thread>>>(aGPU,bGPU,sumGPU);
    cudaMemcpy(sum,sumGPU,size,cudaMemcpyDeviceToHost);
    cudaFree(sumGPU);
    cudaFree(aGPU);
    cudaFree(bGPU);
    return sum;
}

__global__ void kernel(int* a, int N)
{
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < N) {
        int item = a[idx];
        for (int i = 0; i < 100; i++) {
            a[idx] += item;
            
            // ^^^^^ Extra code to evaluate time
            if (idx > 0) {
                a[idx] += a[idx - 1];
            }

            // ^^^^^ Results
            // open: 
            // [INFO] [Regular_callback11]: [PID: 2176175] [kernelCpp] [s: 0] [us: 34]
            // [INFO] [Regular_callback11]: [PID: 2176175] [cudaStreamSynchronize] [s: 0] [us: 18117]
            // close:
            // [INFO] [Regular_callback11]: [PID: 2178310] [kernelCpp] [s: 0] [us: 43]
            // [INFO] [Regular_callback11]: [PID: 2178310] [cudaStreamSynchronize] [s: 0] [us: 1992]
        }
        a[idx] += 2;
    }
}

void kernelCpp(int* h_a, int N, int threadsPerBlock, int blocksPerGrid,
            int* d_a, cudaStream_t stream1)
{
    for (size_t i = 0; i < 100; i++) {
        kernel<<<blocksPerGrid, threadsPerBlock, 0, stream1>>>(d_a, N);
    }
}
    
    

