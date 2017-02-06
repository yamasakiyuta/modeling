#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sys/stat.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //input and output files
  std::string ifile = argv[1];
  std::string ofile;
  //std::string pcd = ".pcd";
  
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
  std::string outliners = "_outliners";
  std::string pcd = ".pcd";
  
  std::string output_inliners = dr_name + "/" + ofile + inliners + pcd;
  std::string output_outliners = dr_name + "/" + ofile + outliners + pcd;
  
  printf("output file[0] : %s\n",output_inliners.c_str());
  printf("output file[1] : %s\n",output_outliners.c_str());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (ifile, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (2);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  
  if(!mkdir("meshes",0755)){
	  printf("/meshes generated\n");
  }
  if(!mkdir(dr_name.c_str(),0755)){
	  printf(" %s\n",dr_name.c_str());
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (output_inliners.c_str(), *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> (output_outliners.c_str(), *cloud_filtered, false);

  return (0);
}
