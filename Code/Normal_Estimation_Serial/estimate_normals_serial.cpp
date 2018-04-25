#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>
#include <sys/time.h>

#define MATCH(s) (!strcmp(argv[ac], (s)))

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  double centroid0 = 0;
  double centroid1 = 0;
  double centroid2 = 0;
  double centroid3 = 0;
  double C_00 = 0;
  double C_01 = 0;
  double C_02 = 0;
  double C_10 = 0;
  double C_11 = 0;
  double C_12 = 0;
  double C_20 = 0;
  double C_21 = 0;
  double C_22 = 0;

  if(argc<2) {
    printf("Usage: %s [-i <inputname>] [-o <outputname>]\n",argv[0]);
    return(-1);
  }
  for(int ac=1;ac<argc;ac++) {
    if(MATCH("-i")) {
      inputname = argv[++ac];
    } else if(MATCH("-o")) {
      outputname = argv[++ac];
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
  
  time_calc_start = get_time();
  if(cloud->isOrganized())
  {
    std::cout << "This cloud is organized!" << std::endl;
    for(int i = 1; i < cloud->width-1; i++)
    {
      for(int j = 1; j < cloud->height-1; j++)
      {
        centroid0 = (cloud->at(i,j).x
                  + cloud->at(i-1,j-1).x
                  + cloud->at(i,j-1).x
                  + cloud->at(i+1,j-1).x
                  + cloud->at(i-1,j).x
                  + cloud->at(i+1,j).x
                  + cloud->at(i-1,j+1).x
                  + cloud->at(i,j+1).x
                  + cloud->at(i+1,j+1).x)/9;
        centroid1 = (cloud->at(i,j).y
                  + cloud->at(i-1,j-1).y
                  + cloud->at(i,j-1).y
                  + cloud->at(i+1,j-1).y
                  + cloud->at(i-1,j).y
                  + cloud->at(i+1,j).y
                  + cloud->at(i-1,j+1).y
                  + cloud->at(i,j+1).y
                  + cloud->at(i+1,j+1).y)/9;
        centroid2 = (cloud->at(i,j).z
                  + cloud->at(i-1,j-1).z
                  + cloud->at(i,j-1).z
                  + cloud->at(i+1,j-1).z
                  + cloud->at(i-1,j).z
                  + cloud->at(i+1,j).z
                  + cloud->at(i-1,j+1).z
                  + cloud->at(i,j+1).z
                  + cloud->at(i+1,j+1).z)/9;
        
        C_00 = ((cloud->at(i-1,j-1).x - centroid0)*(cloud->at(i-1,j-1).x - centroid0)
             + (cloud->at(i,j-1).x - centroid0)*(cloud->at(i,j-1).x - centroid0)
             + (cloud->at(i+1,j-1).x - centroid0)*(cloud->at(i+1,j-1).x - centroid0)
             + (cloud->at(i-1,j).x - centroid0)*(cloud->at(i-1,j).x - centroid0)
             + (cloud->at(i,j).x - centroid0)*(cloud->at(i,j).x - centroid0)
             + (cloud->at(i+1,j).x - centroid0)*(cloud->at(i+1,j).x - centroid0)
             + (cloud->at(i-1,j+1).x - centroid0)*(cloud->at(i-1,j+1).x - centroid0)
             + (cloud->at(i,j+1).x - centroid0)*(cloud->at(i,j+1).x - centroid0)
             + (cloud->at(i+1,j+1).x - centroid0)*(cloud->at(i+1,j+1).x - centroid0))/9;

        C_01 = ((cloud->at(i-1,j-1).x - centroid0)*(cloud->at(i-1,j-1).y - centroid1)
             + (cloud->at(i,j-1).x - centroid0)*(cloud->at(i,j-1).y - centroid1)
             + (cloud->at(i+1,j-1).x - centroid0)*(cloud->at(i+1,j-1).y - centroid1)
             + (cloud->at(i-1,j).x - centroid0)*(cloud->at(i-1,j).y - centroid1)
             + (cloud->at(i,j).x - centroid0)*(cloud->at(i,j).y - centroid1)
             + (cloud->at(i+1,j).x - centroid0)*(cloud->at(i+1,j).y - centroid1)
             + (cloud->at(i-1,j+1).x - centroid0)*(cloud->at(i-1,j+1).y - centroid1)
             + (cloud->at(i,j+1).x - centroid0)*(cloud->at(i,j+1).y - centroid1)
             + (cloud->at(i+1,j+1).x - centroid0)*(cloud->at(i+1,j+1).y - centroid1))/9;

        C_02 = ((cloud->at(i-1,j-1).x - centroid0)*(cloud->at(i-1,j-1).z - centroid2)
             + (cloud->at(i,j-1).x - centroid0)*(cloud->at(i,j-1).z - centroid2)
             + (cloud->at(i+1,j-1).x - centroid0)*(cloud->at(i+1,j-1).z - centroid2)
             + (cloud->at(i-1,j).x - centroid0)*(cloud->at(i-1,j).z - centroid2)
             + (cloud->at(i,j).x - centroid0)*(cloud->at(i,j).z - centroid2)
             + (cloud->at(i+1,j).x - centroid0)*(cloud->at(i+1,j).z - centroid2)
             + (cloud->at(i-1,j+1).x - centroid0)*(cloud->at(i-1,j+1).z - centroid2)
             + (cloud->at(i,j+1).x - centroid0)*(cloud->at(i,j+1).z - centroid2)
             + (cloud->at(i+1,j+1).x - centroid0)*(cloud->at(i+1,j+1).z - centroid2))/9;

        C_10 = C_01;

        C_11 = ((cloud->at(i-1,j-1).y - centroid1)*(cloud->at(i-1,j-1).y - centroid1)
             + (cloud->at(i,j-1).y - centroid1)*(cloud->at(i,j-1).y - centroid1)
             + (cloud->at(i+1,j-1).y - centroid1)*(cloud->at(i+1,j-1).y - centroid1)
             + (cloud->at(i-1,j).y - centroid1)*(cloud->at(i-1,j).y - centroid1)
             + (cloud->at(i,j).y - centroid1)*(cloud->at(i,j).y - centroid1)
             + (cloud->at(i+1,j).y - centroid1)*(cloud->at(i+1,j).y - centroid1)
             + (cloud->at(i-1,j+1).y - centroid1)*(cloud->at(i-1,j+1).y - centroid1)
             + (cloud->at(i,j+1).y - centroid1)*(cloud->at(i,j+1).y - centroid1)
             + (cloud->at(i+1,j+1).y - centroid1)*(cloud->at(i+1,j+1).y - centroid1))/9;

        C_12 = ((cloud->at(i-1,j-1).y - centroid1)*(cloud->at(i-1,j-1).z - centroid2)
             + (cloud->at(i,j-1).y - centroid1)*(cloud->at(i,j-1).z - centroid2)
             + (cloud->at(i+1,j-1).y - centroid1)*(cloud->at(i+1,j-1).z - centroid2)
             + (cloud->at(i-1,j).y - centroid1)*(cloud->at(i-1,j).z - centroid2)
             + (cloud->at(i,j).y - centroid1)*(cloud->at(i,j).z - centroid2)
             + (cloud->at(i+1,j).y - centroid1)*(cloud->at(i+1,j).z - centroid2)
             + (cloud->at(i-1,j+1).y - centroid1)*(cloud->at(i-1,j+1).z - centroid2)
             + (cloud->at(i,j+1).y - centroid1)*(cloud->at(i,j+1).z - centroid2)
             + (cloud->at(i+1,j+1).y - centroid1)*(cloud->at(i+1,j+1).z - centroid2))/9;

        C_20 = C_02;

        C_21 = C_12;

        C_22 = ((cloud->at(i-1,j-1).z - centroid2)*(cloud->at(i-1,j-1).z - centroid2)
             + (cloud->at(i,j-1).z - centroid2)*(cloud->at(i,j-1).z - centroid2)
             + (cloud->at(i+1,j-1).z - centroid2)*(cloud->at(i+1,j-1).z - centroid2)
             + (cloud->at(i-1,j).z - centroid2)*(cloud->at(i-1,j).z - centroid2)
             + (cloud->at(i,j).z - centroid2)*(cloud->at(i,j).z - centroid2)
             + (cloud->at(i+1,j).z - centroid2)*(cloud->at(i+1,j).z - centroid2)
             + (cloud->at(i-1,j+1).z - centroid2)*(cloud->at(i-1,j+1).z - centroid2)
             + (cloud->at(i,j+1).z - centroid2)*(cloud->at(i,j+1).z - centroid2)
             + (cloud->at(i+1,j+1).z - centroid2)*(cloud->at(i+1,j+1).z - centroid2))/9;

        Eigen::Matrix3f C;
        C << C_00, C_01, C_02, C_10, C_11, C_12, C_20, C_21, C_22;

        Eigen::Vector3f::Scalar eigenvalue;
        Eigen::Vector3f eigenvector;
        pcl::eigen33(C, eigenvalue, eigenvector);

        Eigen::Vector3f vp (std::numeric_limits<float>::min () - cloud->at(i,j).x, 
                            std::numeric_limits<float>::min () - cloud->at(i,j).y,
                            std::numeric_limits<float>::min () - cloud->at(i,j).z);

        double flipDecision = vp.dot(eigenvector);

        if(flipDecision < 0)
          eigenvector *= -1;

        outcloud->at(i,j).normal_x = eigenvector[0];
        outcloud->at(i,j).normal_y = eigenvector[1];
        outcloud->at(i,j).normal_z = eigenvector[2];
      }
    }
  }
  else
  {
    return (-1);
  }
  time_calc_end = get_time();

  pcl::io::savePCDFile (outputname, *outcloud);

  time_end = get_time();

  std::cout << "Time it took: " << time_end-time_begin << std::endl;
  std::cout << "Time it took normal estimation: " << time_calc_end - time_calc_start << std::endl;

  return (0);
}