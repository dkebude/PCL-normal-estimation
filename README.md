# PCL-normal-estimation

Parallel Programing course final project: Parallelization of normal estimation in point clouds with and without PCL. 

Comparisons of serial, OpenMP and CUDA implementations can be found below and speed comparisons can be found in the *report.pdf*:

![Alt text](./Supplement/Serial_Comparison.png?raw=true "Serial Code Comparison with and without PCL")

![Alt text](./Supplement/OpenMP_Comparison.png?raw=true "OpenMP version Comparison with and without PCL")

![Alt text](./Supplement/CUDA_nonPCL.png?raw=true "CUDA version Comparison without PCL")

### To compile code:

1. Place code folder (it should include both the code and CMakeLists.txt) under the scratch folder in your parallel programming cluster.
2. cd to code folder (e.g. cd Normal_Estimation_CUDA)
3. mkdir build
4. cd build
5. cmake ..
6. make
7. Run executable to see its usage