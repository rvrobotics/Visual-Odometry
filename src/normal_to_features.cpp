#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub,pub_normal;
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.25, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
	
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ> 
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PCLPointCloud2 cloud_normal;
  //sensor_msgs::PointCloud2 ros_cloud_normal;
  //ros_cloud_normal = *ros_cloud;  
  //pcl_conversions::toPCL(
  pcl::fromROSMsg(*ros_cloud,*cloud);
   //ROS_INFO("%d",cloud->width);
  //pcl::PointCloud<pcl::PointXYZ> cloud_t;
  //pcl::fromPCLPointCloud2(*cloud_normal, *cloud);
  //*cloud = cloud_t;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  //ne.setRadiusSearch (100);
ne.setKSearch(10000); 
  // Compute the features
  ne.compute (*cloud_normals);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  pcl::Normal T;
  T.normal_x = cloud_normals->points[0].normal_x;
  T.normal_y = cloud_normals->points[0].normal_y;
  T.normal_z = cloud_normals->points[0].normal_z;
  T.curvature = cloud_normals->points[0].curvature;
//  cloud_normals1->push_back(T);
  
  pcl::PointCloud<pcl::Normal> cloud1 ;/*(new pcl::PointCloud<pcl::Normal>);/*cloud 1 to be published as
                                                                                    normal of ground plane*/
  pcl::PointXYZ P;
  P.x = T.normal_x;//cloud->points[0].x;
  P.y = T.normal_y;//cloud->points[0].y;
  P.z = T.normal_z;//cloud->points[0].z;
  //cloud1->push_back(P);

//  pcl::PointXYZ P2;
//  P2.x = T.normal_x;
//  P2.y = T.normal_y;
//  P2.z = T.normal_z;
  for(long int i=0;i<cloud_normals->points.size();i++){
  	if(cloud_normals->at(i).normal[2] > 0.0){
      ROS_ERROR("%f %f %f %f",
                cloud_normals->at(i).normal[0],
                cloud_normals->at(i).normal[1],
                cloud_normals->at(i).normal[2],
                cloud_normals->at(i).curvature
                );
      T.normal_x = cloud_normals->at(i).normal[0];
      T.normal_y = cloud_normals->at(i).normal[1];
      T.normal_z = cloud_normals->at(i).normal[2];
      T.curvature = cloud_normals->at(i).curvature;
      cloud1.push_back(T);
      break;
	  //ROS_ERROR("%f %f %f ",cloud_normals->points[i].normal_x,cloud_normals->points[i].normal_y,cloud_normals->points[i].normal_z);
	}
	  //ROS_ERROR("%f %f %f ",cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
	  //ROS_ERROR("%f %f %f ",T.normal_x,T.normal_y,T.normal_z,T.curvature);
  }
//  pcl::PointXYZ P2;
//  P2.x = T.normal_x;
//  P2.y = T.normal_y;
//  P2.z = T.normal_z;
//  cloud1->push_back(T);

//  pcl::PCLPointCloud2 normal;
//  pcl::toPCLPointCloud2(*cloud1 , normal);
//  sensor_msgs::PointCloud2 surface_normal;
//  pcl_conversions::fromPCL(normal,surface_normal);

  pub_normal.publish(cloud1);

  pcl::PCLPointCloud2 cloud_final;
  pcl::toPCLPointCloud2(*cloud_normals,cloud_final);
  
  sensor_msgs::PointCloud2 ros_cloud_normal2;
  pcl_conversions::fromPCL(cloud_final,ros_cloud_normal2);
  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  pub.publish(ros_cloud_normal2);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0.0, 0.0, 0.5);

  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
 
  //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals,10,0.5,"normals");
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//  viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(P,P2,255,0,0);
  while (!viewer->wasStopped ())
  {
     viewer->spinOnce ();
  }
  
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  ros::Subscriber sub = n.subscribe("/kitti_stereo/points2", 1, Callback);

  pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud2_normal", 1);
  pub_normal = n.advertise<pcl::PointCloud<pcl::Normal> > ("surface_normal",1);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
//  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  ros::spin();

  return 0;
}
