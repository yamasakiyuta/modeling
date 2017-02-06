#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sys/stat.h>
#include <pcl/features/normal_3d.h>
#include <time.h>
int
main (int argc, char** argv)
{
  clock_t start_time = time(NULL);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

  //input and output files
  std::string ifile = argv[1];
  std::string ofile;
  
  int path_i = ifile.find_last_of("/")+1;
  int ext_i = ifile.find_last_of(".");
  std::string pathname = ifile.substr(0,path_i+1);
  std::string extname = ifile.substr(ext_i,ifile.size()-ext_i);
  std::string filename = ifile.substr(path_i,ext_i-path_i);
  //printf("%s\n",filename.c_str()); 
  
  if(argc>3 || argc<1){
	  printf("error\n");
	  return 0;
  }
  else if(argc<3){
	  //printf("ofile\n");
	  ofile = filename;
  }
  else ofile = argv[2];
  
  printf("input file : %s\n", argv[1]); 
 
  std::string dr_name = "meshes/" + ofile;
  std::string inliners = "_inliners";
  std::string pcd = ".pcd";
  
  std::string output_inliners = dr_name + "/" + ofile + inliners + pcd;
  
  printf("output file[0] : %s\n",output_inliners.c_str());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (ifile, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  std::cerr << "[" << difftime(time(NULL),start_time) << "sec]" << "filtering: 1/3" << std::endl;
  
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (2);
  sor.filter (*cloud_filtered);
  
  std::cerr << "[" << difftime(time(NULL),start_time) << "sec]" << "filtering: 2/3" << std::endl;
  
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (2);
  sor.filter (*cloud_filtered);
  
  std::cerr << "[" << difftime(time(NULL),start_time) << "sec]" << "filtering: 3/3" << std::endl;
  
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (2);
  sor.filter (*cloud_filtered);
  
  std::cerr << "[" << difftime(time(NULL),start_time) << "sec]" << "estimating normals" << std::endl;

 // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 5cm
  ne.setRadiusSearch (0.5);

  ne.setViewPoint(0.0,0.0,0.0);
  // Compute the features
  ne.compute (*cloud_normals);

  pcl::concatenateFields (*cloud_filtered, *cloud_normals, *cloud_with_normals);
  
  std::cerr << "[" << difftime(time(NULL),start_time) << "sec]" << "finish" << std::endl;
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  
  if(!mkdir("meshes",0755)){
	  printf("/meshes generated\n");
  }
  if(!mkdir(dr_name.c_str(),0755)){
	  printf(" %s\n",dr_name.c_str());
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> (output_inliners.c_str(), *cloud_with_normals, false);

  return (0);
}
