#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/cuda/common/eigen.h>
#include <sys/time.h>

#define MATCH(s) (!strcmp(argv[ac], (s)))

extern void kernel_wrapper(float *points_x, float *points_y, float *points_z, float *normal_x, float *normal_y, float *normal_z, int width, int height, int size, int block_size);

// returns the current time
static const double kMicro = 1.0e-6;
double get_time() {
  struct timeval TV;
  struct timezone TZ;
  const int RC = gettimeofday(&TV, &TZ);
  if(RC == -1) {
    printf("ERROR: Bad call to gettimeofday\n");
    return(-1);
  }
  return( ((double)TV.tv_sec) + kMicro * ((double)TV.tv_usec) );
}

int
main (int argc, char** argv)
{
  double time_begin, time_end, time_calc_start, time_calc_end;
  time_begin = get_time();
  const char *inputname = "Data/demo0_kf0.pcd";
  const char *outputname = "Data/demo0_kf0-normals.pcd";
  int block_size = 16;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  if(argc<2) {
    printf("Usage: %s [-i <inputname>] [-o <outputname>]\n",argv[0]);
    return(-1);
  }
  for(int ac=1;ac<argc;ac++) {
    if(MATCH("-i")) {
      inputname = argv[++ac];
    } else if(MATCH("-o")) {
      outputname = argv[++ac];
    } else if(MATCH("-b")) {
      block_size = atoi(argv[++ac]);
    } else {
      printf("Usage: %s [-i <inputname>] [-o <outputname>]\n",argv[0]);
      return(-1);
    }
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (inputname, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", inputname);
    return (-1);
  }
  std::cout << "Loaded cloud with data count: "
            << cloud->width * cloud->height
            << " with width: "
            << cloud->width
            << " and height: "
            << cloud->height
            << std::endl << std::endl;
  pcl::copyPointCloud(*cloud, *outcloud);

  std::cout << "Trying to copy points to device memory" << std::endl;

  int width = cloud->width;
  int height = cloud->height;
  int size = width*height;
  float *points_x = (float*) malloc (sizeof(float)*size);
  float *points_y = (float*) malloc (sizeof(float)*size);
  float *points_z = (float*) malloc (sizeof(float)*size);
  float *normal_x = (float*) malloc (sizeof(float)*size);
  float *normal_y = (float*) malloc (sizeof(float)*size);
  float *normal_z = (float*) malloc (sizeof(float)*size);

  for(int i = 0; i < height; i++)
  {
    for(int j = 0; j < width; j++)
    {
      points_x[i*width+j] = cloud->points[i*width+j].x;
      points_y[i*width+j] = cloud->points[i*width+j].y;
      points_z[i*width+j] = cloud->points[i*width+j].z;
    }
  }

  
  if(cloud->isOrganized())
  {
    std::cout << "This cloud is organized!" << std::endl;

    time_calc_start = get_time();
    kernel_wrapper(points_x, points_y, points_z, normal_x, normal_y, normal_z, width, height, size, block_size);
    time_calc_end = get_time();
  }
  else
  {
    return (-1);
  }

  for(int i = 0; i < height; i++)
  {
    for(int j = 0; j < width; j++)
    {
      outcloud->points[i*width+j].normal_x = normal_x[i*width+j];
      outcloud->points[i*width+j].normal_y = normal_y[i*width+j];
      outcloud->points[i*width+j].normal_z = normal_z[i*width+j];
    }
  }  
  
  pcl::io::savePCDFile (outputname, *outcloud);

  time_end = get_time();

  std::cout << "Time it took: " << time_end-time_begin << std::endl;
  std::cout << "Time it took normal estimation: " << time_calc_end - time_calc_start << std::endl;

  return (0);
}
