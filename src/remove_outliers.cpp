#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <sys/stat.h>
#include <pcl/features/normal_3d.h>
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
 
  std::string dr_name = "outlier_remove/" + ofile;
  std::string tag = "_outlier";
  std::string pcd = ".pcd";
  
  std::string output_tag = dr_name + "/" + ofile + tag + pcd;
  
  printf("output file[0] : %s\n",output_tag.c_str());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (ifile, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.1);
  outrem.setMinNeighborsInRadius (0);
  // apply filter
  outrem.filter (*cloud_filtered);

  if(!mkdir("outlier_remove",0755)){
	  printf("/outlier_remove generated\n");
  }
  if(!mkdir(dr_name.c_str(),0755)){
	  printf(" %s\n",dr_name.c_str());
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (output_tag.c_str(), *cloud_filtered, false);

  return (0);
}
