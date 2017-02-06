#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_exports.h>
#include <sys/stat.h>

int
main (int argc, char** argv)
{
  //printf("argc = %d\n", argc);
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  
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
  std::string vtk = ".vtk";
  std::string obj = ".obj";
  std::string ply = ".ply";
  
  std::string output_vtk = dr_name + "/" + ofile + vtk;
  std::string output_obj = dr_name + "/" + ofile + obj;
  std::string output_ply = dr_name + "/" + ofile + ply;
  
  printf("output file : %s\n",output_vtk.c_str());
  
  pcl::io::loadPCDFile (ifile, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  printf("normal estimation\n"); 
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (1.5);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

 printf("get result\n"); 
  // Get result
  gp3.setInputCloud (cloud_with_normals);
 printf("search method\n"); 
  gp3.setSearchMethod (tree2);
 printf("reconstruct\n"); 
  gp3.reconstruct (triangles);
 printf("writing\n"); 
  
  if(!mkdir("meshes",0755)){
	  printf("ディレクトリを作成\n");
  }
  if(!mkdir(dr_name.c_str(),0755)){
	  printf("ディレクトリを作成 %s\n",dr_name.c_str());
  }
  pcl::io::saveVTKFile (output_vtk, triangles);
  pcl::io::savePLYFile (output_ply, triangles);
  //pcl::io::saveOBJFile (output_obj, triangles);
  //pcl::savePolygonFileVTK ("VTK_mesh.vtk", triangles);
  
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
 printf("finished\n");
  // Finish
  return (0);
}
