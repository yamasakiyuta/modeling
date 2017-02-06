#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/cloud_viewer.h>


pcl::PointCloud<pcl::Normal>::Ptr surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);//法線の計算を行いたい点群を指定する

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());//KDTREEを作る
    ne.setSearchMethod (tree);//検索方法にKDTREEを指定する

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//法線情報を入れる変数

    //ne.setRadiusSearch (0.005);//検索する半径を指定する
    ne.setRadiusSearch (0.5);//検索する半径を指定する

    ne.compute (*cloud_normals);//法線情報の出力先を指定する

    return cloud_normals;
}

pcl::PolygonMesh Generate_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    cloud_normals = surface_normals(cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    //gp3.setSearchRadius (0.025);
    gp3.setSearchRadius (0.2);

    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); 
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setNormalConsistency(false);

    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;
}

int main(int argc, char* argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);

    std::stringstream Filename;
    //string name = argv[1];
    //name.erase(name.length() - 4);
    //Filename << name << "_mesh.vtk";
    //pcl::io::saveVTKFile(Filename.str(), Generate_mesh(cloud));
    //pcl::io::savePCDFile("ply_a.pcd", Generate_mesh(cloud));
 pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
viewer.addPolygonMesh(Generate_mesh(cloud));
  while (!viewer.wasStopped ())
    {
  viewer.spinOnce(10);

 // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }   
    return (0);
}
