#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sys/time.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/initialization.h>

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
  double time_begin, time_end, time_calc_start, time_calc_end, time_ne_start, time_ne_end;
  time_begin = get_time();
  const char *inputname = "input.pgm";
  const char *outputname = "output.png";
  double radius = 0.03;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr work_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if(argc<2) {
    printf("Usage: %s [-i <inputname>] [-r <radius>] [-o <outputname>]\n",argv[0]);
    return(-1);
  }
  for(int ac=1;ac<argc;ac++) {
    if(MATCH("-i")) {
      inputname = argv[++ac];
    } else if(MATCH("-r")) {
      radius = atof(argv[++ac]);
    } else if(MATCH("-o")) {
      outputname = argv[++ac];
    } else {
      printf("Usage: %s [-i <inputname>] [-r <radius>] [-o <outputname>]\n",argv[0]);
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
  pcl::gpu::Feature::PointCloud cloud_d(cloud->width * cloud->height);
  pcl::copyPointCloud(*cloud, *work_cloud);
  time_ne_start = get_time();
  cloud_d.upload(work_cloud->points);  
  
  // pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;  
  // if(cloud->isOrganized())
  // {
  //   std::cout << "This cloud is organized!" << std::endl;
  //   tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
  // }
  // else
  // {
  //   std::cout << "This cloud is not organized!" << std::endl << std::endl;
  //   tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
  // }
  // tree->setInputCloud (cloud);

  pcl::gpu::NormalEstimation ne_d; 
  ne_d.setInputCloud (cloud_d);
  ne_d.setViewPoint (std::numeric_limits<float>::min (), std::numeric_limits<float>::min (), std::numeric_limits<float>::min ());
  ne_d.setRadiusSearch (radius, 8);

  pcl::gpu::Feature::Normals normals_d(cloud->width * cloud->height);
  std::vector<pcl::PointXYZ> normals;
  time_calc_start = get_time();
  ne_d.compute(normals_d);
  time_calc_end = get_time();
  normals_d.download(normals);
  time_ne_end = get_time();

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud (*cloud, *cloud_with_normals);

  for(int i = 0; i < normals.size(); i++)
  {
    cloud_with_normals->points[i].normal_x = normals[i].x;
    cloud_with_normals->points[i].normal_y = normals[i].y;
    cloud_with_normals->points[i].normal_z = normals[i].z;
  }

  pcl::io::savePCDFile (outputname, *cloud_with_normals);

  // for (size_t i = 0; i < cloud->points.size (); ++i)
  //   std::cout << "    " << cloud->points[i].x
  //             << " "    << cloud->points[i].y
  //             << " "    << cloud->points[i].z << std::endl;
  time_end = get_time();

  std::cout << "Time it took: " << time_end - time_begin << std::endl;
  std::cout << "Time it took for compute: " << time_calc_end - time_calc_start << std::endl;
  std::cout << "Time it took for normal estimation" << time_ne_end - time_ne_start << std::endl;

  return (0);
}