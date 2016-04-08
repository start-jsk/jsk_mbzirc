/*
 jsk_mbzirc_task
 cuda accelerate
 */

// Author: Chen

//opencv
#include <cv_bridge/cv_bridge.h>

//cuda
#include <include/cuda_runtime.h>
#include <include/cuda.h>
//pcl
#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#define Ground_Z 0.0
typedef pcl::PointXYZRGB PointTYPE;


static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
}
#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

__global__ void cuda_projecting(double *a, double *b, double *c, uchar *imgdata, float *pcdata)
{
    float A[2][2],bv[2];
//    int i = blockIdx.x; //row
//    int j = blockIdx.y; //column
//    int offset = j + i*gridDim.y;
    int i = blockIdx.x;
    int j = threadIdx.x;
    int offset = j + i*blockDim.x;
    A[0][0] = j*c[0] - a[0]; A[0][1] = j*c[1] - a[1];
    A[1][0] = i*c[0] - b[0]; A[1][1] = i*c[1] - b[1];
    bv[0]= a[2]*Ground_Z + a[3] - j*c[2]*Ground_Z - j*c[3];
    bv[1] = b[2]*Ground_Z + b[3] - i*c[2]*Ground_Z - i*c[3];
    float DomA = A[1][1]*A[0][0]-A[0][1]*A[1][0];
    int offsetimg = offset*3;  //3 channel
    int offsetpc = offset*( sizeof(PointTYPE)/sizeof(float));
    pcdata[offsetpc] = (A[1][1]*bv[0]-A[0][1]*bv[1])/DomA;
    pcdata[offsetpc+1] = (A[0][0]*bv[1]-A[1][0]*bv[0])/DomA;
    pcdata[offsetpc+2] = (float)Ground_Z;
    uint8_t rgb[4];
    rgb[0] = imgdata[offsetimg]; rgb[1] = imgdata[offsetimg+1];
    rgb[2] = imgdata[offsetimg+2]; rgb[3] = 0;
    pcdata[offsetpc+3] = 1.0;
    pcdata[offsetpc+4] = *(float *)(rgb);
}

float process_in_cuda(double *_a, double *_b,double *_c,
                    cv::Mat *_img, pcl::PointCloud<PointTYPE> *PC)
{
    uchar *dev_imgdata;
    float *dev_pcdata;
    const int coefsize = 4*sizeof(double);
    const int pixelsize = _img->rows*_img->cols;
    const int pointcloudsize = pixelsize*sizeof(PointTYPE);
    double *dev_a, *dev_b, *dev_c;

    //malloc the mem
    cudaSetDevice(0);
    HANDLE_ERROR(cudaMalloc((void **)&dev_a, coefsize));
    HANDLE_ERROR(cudaMalloc((void **)&dev_b, coefsize));
    HANDLE_ERROR(cudaMalloc((void **)&dev_c, coefsize));
    HANDLE_ERROR(cudaMalloc((void **)&dev_imgdata,sizeof(uchar)*pixelsize*_img->channels()));
    HANDLE_ERROR(cudaMalloc((void **)&dev_pcdata, pixelsize*sizeof(PointTYPE)));

    //copy to device mem
    HANDLE_ERROR(cudaMemcpy(dev_a,_a,coefsize,cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_b,_b,coefsize,cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_c,_c,coefsize,cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(dev_imgdata,_img->data,
                            sizeof(uchar)*pixelsize*_img->channels(),cudaMemcpyHostToDevice));

    //create cudaevent to record the time...
    cudaEvent_t start,stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);

    //run device function...
//    dim3 grid(PC->height,PC->width);
//    cuda_projecting<<<grid,1>>>(dev_a, dev_b, dev_c, dev_imgdata, dev_pcdata);
    cuda_projecting<<<PC->height,PC->width>>>(dev_a, dev_b, dev_c, dev_imgdata, dev_pcdata);
    //copy back to pointcloud...
    HANDLE_ERROR(cudaMemcpy(PC->points.data(),dev_pcdata,pointcloudsize,cudaMemcpyDeviceToHost));
    cudaEventRecord( stop, 0 );

    float   elapsedTime;
    cudaEventElapsedTime(&elapsedTime,start,stop);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    cudaFree(dev_a); cudaFree(dev_b); cudaFree(dev_c);
    cudaFree(dev_imgdata); cudaFree(dev_pcdata);

   return elapsedTime;

}
