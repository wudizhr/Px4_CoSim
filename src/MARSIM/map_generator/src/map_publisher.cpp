#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/common.h>

// #include <pcl/point_types.h>
#include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/impl/kdtree.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>

typedef pcl::PointXYZ PointT;

using namespace std;
string file_name;

int add_boundary = 0;
int is_bridge = 0;
double downsample_res;
double map_offset_x,map_offset_y,map_offset_z;
string frame_id = "world";

int minus_twopointcloud(pcl::PointCloud<pcl::PointXYZ>& cloud_input, pcl::PointCloud<pcl::PointXYZ>& cloud_input2, pcl::PointCloud<pcl::PointXYZ>& cloud_output)
{
  //cloud_input minus cloud_input2
  pcl::search::KdTree<pcl::PointXYZ> minus_kdtree;
  minus_kdtree.setInputCloud(cloud_input2.makeShared());

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  for(int i = 0; i < cloud_input.points.size();i++)
  {
    if (minus_kdtree.radiusSearch(cloud_input.points[i], 0.6, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      continue;
    }
    cloud_output.points.push_back(cloud_input.points[i]);
  }
  
  // ROS_INFO replaced with cout for now
  std::cout << "AFTER MINUS, points count = " << cloud_output.points.size() << std::endl;

  return cloud_output.points.size();

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("map_recorder");

  node->declare_parameter("add_boundary", 0);
  node->declare_parameter("is_bridge", 0);
  node->declare_parameter("downsample_res", 0.1);
  node->declare_parameter("map_offset_x", 0.0);
  node->declare_parameter("map_offset_y", 0.0);
  node->declare_parameter("map_offset_z", 0.0);
  node->declare_parameter("frame_id", std::string("world"));
  node->declare_parameter("pcd_file", std::string(""));

  add_boundary = node->get_parameter("add_boundary").as_int();
  is_bridge = node->get_parameter("is_bridge").as_int();
  downsample_res = node->get_parameter("downsample_res").as_double();
  map_offset_x = node->get_parameter("map_offset_x").as_double();
  map_offset_y = node->get_parameter("map_offset_y").as_double();
  map_offset_z = node->get_parameter("map_offset_z").as_double();
  frame_id = node->get_parameter("frame_id").as_string();

  file_name = node->get_parameter("pcd_file").as_string();
  if (file_name.empty()) {
    if (argc > 1) {
      file_name = argv[1];
    } else {
      RCLCPP_ERROR(node->get_logger(), "No map file provided. Set parameter 'pcd_file' in launch or pass argv[1].");
      rclcpp::shutdown();
      return -1;
    }
  }

  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 10);

  rclcpp::sleep_for(std::chrono::seconds(1));

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud_temp, cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud_temp);
  cloud = cloud_temp;

  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "SUCCESS LOAD PCD FILE");

  //filter
  pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;

  if(is_bridge == 1)
  {
  // Process map
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x;
    pr.y = -pt.z;
    pr.z = pt.y;
    cloud.points[i] = pr;
  }    
  }

  pcl::PointXYZ global_mapmin;
  pcl::PointXYZ global_mapmax;
  
  pcl::getMinMax3D(cloud,global_mapmin,global_mapmax);

  RCLCPP_INFO(node->get_logger(), "Map bound: x=%f,%f, y=%f,%f, z=%f,%f",global_mapmin.x,global_mapmax.x,global_mapmin.y,global_mapmax.y,global_mapmin.z,global_mapmax.z);

  if(add_boundary == 1)
  {
  //add boundary
  pcl::PointCloud<pcl::PointXYZ> cloud_boundary;
  int add_length = 1;
  int max_x = (int)global_mapmax.x + add_length;
  int min_x = (int)global_mapmin.x - add_length;
  int max_y = (int)global_mapmax.y + add_length;
  int min_y = (int)global_mapmin.y - add_length;
  int max_z = (int)global_mapmax.z + add_length;
  int min_z = (int)global_mapmin.z - add_length;
  int box_x = (max_x - min_x)*10;
  int box_y = (max_y - min_y)*10;
  int box_z = (max_z - min_z)*10;
  //draw six plane
  cloud_boundary.width = (box_x+1) * (box_y+1) *(box_z+1) - (box_x-1) * (box_y-1) *(box_z-1);//points number
  cloud_boundary.height=1;
  cloud_boundary.points.resize(cloud_boundary.width*cloud_boundary.height);
  int point_count = 0;
  //draw 2 xy plane
  for (float i = min_x;i <= max_x;i = i+0.1)
  {
    for (float j = min_y;j <= max_y;j = j+0.1)
    {
      cloud.push_back(pcl::PointXYZ(i, j, min_z));
      cloud.push_back(pcl::PointXYZ(i, j, max_z));
    }
  }
  //draw 4 plane
  for (float k = min_z;k <= max_z;k = k+0.1)
  {
    //draw x two lines
    for (float i = min_x;i <= max_x;i = i+0.1)
    {
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = min_y;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = max_y;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = min_y-0.1;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = max_y+0.1;
      cloud_boundary.points[point_count].z = k;
      point_count++;
    }
    for (float j = min_y; j <= max_y; j =j+0.1)
    {
      cloud_boundary.points[point_count].x = min_x;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = max_x;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = min_x-0.1;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = max_x+0.1;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
    }
  }

  cloud = cloud_boundary+cloud;

  std::cout << "ADD BOUNDARY!!!" << std::endl;
  }

  // pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud.makeShared());
  _voxel_sampler.filter(cloud);

  // Process map
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x + map_offset_x;
    pr.y = pt.y + map_offset_y;
    pr.z = pt.z + map_offset_z;
    cloud.points[i] = pr;
  }
  //   // // 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
    pcl::PolygonMesh triangles; //存储最终三角化的网络模型

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  RCLCPP_INFO(node->get_logger(), "Map point size = %d", (int)cloud.points.size());

  int count = 0;
  while (rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    cloud_pub->publish(msg);
    ++count;
    if (count > 10)
    {
      break;
    }
  }
  cout << "finish publish map." << endl;

  rclcpp::shutdown();
  return 0;
}