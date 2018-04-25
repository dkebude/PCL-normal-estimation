#include <pcl/cuda/common/eigen.h>
#include <iostream>

// returns the current time
extern double get_time();

__global__ void compute(float *d_points_x, float *d_points_y, float *d_points_z, float *d_normal_x, float *d_normal_y, float *d_normal_z, int width, int height, int size)
{
  double centroid0 = 0;
  double centroid1 = 0;
  double centroid2 = 0;
  double C_00 = 0;
  double C_01 = 0;
  double C_02 = 0;
  double C_10 = 0;
  double C_11 = 0;
  double C_12 = 0;
  double C_20 = 0;
  double C_21 = 0;
  double C_22 = 0;

  int i = blockIdx.x*blockDim.x + threadIdx.x;
  int j = blockIdx.y*blockDim.y + threadIdx.y;
  int idx = j*width+i;
  int NWidx = (j-1)*width+(i-1);
  int Nidx = (j-1)*width+i;
  int NEidx = (j-1)*width+(i+1);
  int Widx = j*width+(i-1);
  int Eidx = j*width+(i+1);
  int SWidx = (j+1)*width+(i-1);
  int Sidx = (j+1)*width+i;
  int SEidx = (j+1)*width+(i+1);

  if(i >= 0 && j >= 0 && j < height && i < width)
  {
    float NWx = d_points_x[NWidx];
    float NWy = d_points_y[NWidx];
    float NWz = d_points_z[NWidx];
    float Nx  = d_points_x[Nidx];
    float Ny  = d_points_y[Nidx];
    float Nz  = d_points_z[Nidx];
    float NEx = d_points_x[NEidx];
    float NEy = d_points_y[NEidx];
    float NEz = d_points_z[NEidx];
    float Wx  = d_points_x[Widx];
    float Wy  = d_points_y[Widx];
    float Wz  = d_points_z[Widx];
    float x   = d_points_x[idx];
    float y   = d_points_y[idx];
    float z   = d_points_z[idx];
    float Ex  = d_points_x[Eidx];
    float Ey  = d_points_y[Eidx];
    float Ez  = d_points_z[Eidx];
    float SWx = d_points_x[SWidx];
    float SWy = d_points_y[SWidx];
    float SWz = d_points_z[SWidx];
    float Sx  = d_points_x[Sidx];
    float Sy  = d_points_y[Sidx];
    float Sz  = d_points_z[Sidx];
    float SEx = d_points_x[SEidx];
    float SEy = d_points_y[SEidx];
    float SEz = d_points_z[SEidx];

    centroid0 = (NWx+Nx+NEx+Wx+x+Ex+SWx+Sx+SEx)/9;
    centroid1 = (NWy+Ny+NEy+Wy+y+Ey+SWy+Sy+SEy)/9;
    centroid2 = (NWz+Nz+NEz+Wz+z+Ez+SWz+Sz+SEz)/9;

    C_00 = ((NWx - centroid0)*(NWx - centroid0)
          + (Nx  - centroid0)*(Nx  - centroid0)
          + (NEx - centroid0)*(NEx - centroid0)
          + (Wx  - centroid0)*(Wx  - centroid0)
          + (x   - centroid0)*(x   - centroid0)
          + (Ex  - centroid0)*(Ex  - centroid0)
          + (SWx - centroid0)*(SWx - centroid0)
          + (Sx  - centroid0)*(Sx  - centroid0)
          + (SEx - centroid0)*(SEx - centroid0))/9;

    C_01 = ((NWx - centroid0)*(NWy - centroid1)
          + (Nx  - centroid0)*(Ny  - centroid1)
          + (NEx - centroid0)*(NEy - centroid1)
          + (Wx  - centroid0)*(Wy  - centroid1)
          + (x   - centroid0)*(y   - centroid1)
          + (Ex  - centroid0)*(Ey  - centroid1)
          + (SWx - centroid0)*(SWy - centroid1)
          + (Sx  - centroid0)*(Sy  - centroid1)
          + (SEx - centroid0)*(SEy - centroid1))/9;

    C_02 = ((NWx - centroid0)*(NWz - centroid2)
          + (Nx  - centroid0)*(Nz  - centroid2)
          + (NEx - centroid0)*(NEz - centroid2)
          + (Wx  - centroid0)*(Wz  - centroid2)
          + (x   - centroid0)*(z   - centroid2)
          + (Ex  - centroid0)*(Ez  - centroid2)
          + (SWx - centroid0)*(SWz - centroid2)
          + (Sx  - centroid0)*(Sz  - centroid2)
          + (SEx - centroid0)*(SEz - centroid2))/9;

    C_10 = C_01;
    
    C_11 = ((NWy - centroid1)*(NWy - centroid1)
          + (Ny  - centroid1)*(Ny  - centroid1)
          + (NEy - centroid1)*(NEy - centroid1)
          + (Wy  - centroid1)*(Wy  - centroid1)
          + (y   - centroid1)*(y   - centroid1)
          + (Ey  - centroid1)*(Ey  - centroid1)
          + (SWy - centroid1)*(SWy - centroid1)
          + (Sy  - centroid1)*(Sy  - centroid1)
          + (SEy - centroid1)*(SEy - centroid1))/9;
    
    C_12 = ((NWy - centroid1)*(NWz - centroid2)
         + (Ny   - centroid1)*(Nz  - centroid2)
         + (NEy  - centroid1)*(NEz - centroid2)
         + (Wy   - centroid1)*(Wz  - centroid2)
         + (y    - centroid1)*(z   - centroid2)
         + (Ey   - centroid1)*(Ez  - centroid2)
         + (SWy  - centroid1)*(SWz - centroid2)
         + (Sy   - centroid1)*(Sz  - centroid2)
         + (SEy  - centroid1)*(SEz - centroid2))/9;
    
    C_20 = C_02;
    
    C_21 = C_12;
    
    C_22 = ((NWz - centroid2)*(NWz - centroid2)
          + (Nz  - centroid2)*(Nz  - centroid2)
          + (NEz - centroid2)*(NEz - centroid2)
          + (Wz  - centroid2)*(Wz  - centroid2)
          + (z   - centroid2)*(z   - centroid2)
          + (Ez  - centroid2)*(Ez  - centroid2)
          + (SWz - centroid2)*(SWz - centroid2)
          + (Sz  - centroid2)*(Sz  - centroid2)
          + (SEz - centroid2)*(SEz - centroid2))/9;

    pcl::cuda::CovarianceMatrix C;
    C.data[0].x = C_00;
    C.data[0].y = C_01;
    C.data[0].z = C_02;
    C.data[1].x = C_10;
    C.data[1].y = C_11;
    C.data[1].z = C_12;
    C.data[2].x = C_20;
    C.data[2].y = C_21;
    C.data[2].z = C_22;
     
    float3 eigenvalue;
    pcl::cuda::CovarianceMatrix eigenvector;
    pcl::cuda::eigen33(C, eigenvector, eigenvalue);
    float3 vp;  
    vp.x = 1.17549e-38 - x;
    vp.y = 1.17549e-38 - y;
    vp.z = 1.17549e-38 - z;
    float3 normal = normalize(eigenvector.data[0]); 

    double flipDecision = dot(vp,normal);
    if(flipDecision < 0)
      normal *= -1;
    
    d_normal_x[idx] = normal.x;
    d_normal_y[idx] = normal.y;
    d_normal_z[idx] = normal.z;
  }
}

void kernel_wrapper(float *points_x, float *points_y, float *points_z, float *normal_x, float *normal_y, float *normal_z, int width, int height, int size, int block_size)
{
  double time_begin, time_end;

  float *d_points_x;
  float *d_points_y;
  float *d_points_z;
  float *d_normal_x;
  float *d_normal_y;
  float *d_normal_z;

  cudaMalloc((void**) &d_points_x, (sizeof(float)*size));
  cudaMalloc((void**) &d_points_y, (sizeof(float)*size));
  cudaMalloc((void**) &d_points_z, (sizeof(float)*size));
  cudaMalloc((void**) &d_normal_x, (sizeof(float)*size));
  cudaMalloc((void**) &d_normal_y, (sizeof(float)*size));
  cudaMalloc((void**) &d_normal_z, (sizeof(float)*size));
  cudaMemcpy(d_points_x, points_x, (sizeof(float)*size), cudaMemcpyHostToDevice);
  cudaMemcpy(d_points_y, points_y, (sizeof(float)*size), cudaMemcpyHostToDevice);
  cudaMemcpy(d_points_z, points_z, (sizeof(float)*size), cudaMemcpyHostToDevice);

  int ntx = block_size, nty = block_size;
  dim3 threads(ntx, nty);
  dim3 grid(width/ntx, height/nty);
  time_begin = get_time();
  compute<<<grid, threads>>>(d_points_x, d_points_y, d_points_z, d_normal_x, d_normal_y, d_normal_z, width, height, size);
  time_end = get_time();

  cudaMemcpy(normal_x, d_normal_x, (sizeof(float)*size), cudaMemcpyDeviceToHost);
  cudaMemcpy(normal_y, d_normal_y, (sizeof(float)*size), cudaMemcpyDeviceToHost);
  cudaMemcpy(normal_z, d_normal_z, (sizeof(float)*size), cudaMemcpyDeviceToHost);

  std::cout << "Time it took to compute: " << time_end - time_begin << std::endl;
}
