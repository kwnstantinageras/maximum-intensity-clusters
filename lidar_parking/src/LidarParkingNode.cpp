//my .h files
#include "LidarParkingNode.h"

//PCL 
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/projection_matrix.h>

//C++
#include <iostream>
#include <chrono>
#include <limits>

using namespace std;

static const double point_intensity_max_difference = 0.9f;
static const double leaf_size_x = 0.1f;
static const double leaf_size_y = 0.1f;
static const double leaf_size_z = 0.1f;
static const double cluster_tolerance = 0.1f;
static const int minimum_cluster_size = 20; 
static const int maximum_cluster_size = 40;

//Intensity condition for clustering
bool renforceIntensitySimilarity(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance){
      //TODO: determine what is the best difference for our case
      if (std::abs (point_a.intensity - point_b.intensity) < point_intensity_max_difference){
        return (true);
      }
      else{
        return (false);
      }
           
    
}

//CONSTRUCTOR
LidarParkingNode::LidarParkingNode():Node("Lidar_parking"){

  start_parking_ = this->create_subscription<std_msgs::msg::Bool>("/start_parking", 10, std::bind(&LidarParkingNode::start_parking_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Subscribed to topic \"/start_parking\", waiting for \"true\" message..");

  
  testing_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/testing_results", 10);

  reflectors_distances_publisher_ = this->create_publisher<clustering_interfaces::msg::DistanceToClusters>("/reflectors_distances", 10);

  cluster_one_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_one_points", 10);
  cluster_two_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_two_points", 10);

}


// PointCloud topic callback
void LidarParkingNode::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    if(start_parking == true){
       RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received \"PointCloud2\" msg.");
    }
   
    unsigned int num_points = msg->width;
    //RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *point_cloud);


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZI> vg;  
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud (point_cloud);
    vg.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //IDEA cloud_filtered is pcl::pointcloud format with filter



    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    

    // Clustering
    pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
    cec.setInputCloud (cloud_filtered); // cloud_filtered is the pointcloud now used
    cec.setConditionFunction (&renforceIntensitySimilarity);
    //ADDED Kd-tree search method
    cec.setSearchMethod(tree);
    // Points within this distance from one another are going to need to validate the enforceIntensitySimilarity function to be part of the same cluster:
    cec.setClusterTolerance (cluster_tolerance); //TODO: determine what is the best distance tolerance for our case
    // Size constraints for the clusters:
    cec.setMinClusterSize (minimum_cluster_size);
    cec.setMaxClusterSize (maximum_cluster_size);

    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    // The resulting clusters (an array of pointindices):
    cec.segment (*clusters);    //~~~~~~~~~returns IndicesClusters(which are std::vector<pcl::PointIndices>)
    // pcl::PointIndices is struct and has as public attribute Indices indices, where pcl::IndicesAllocator = typedef std::vector<index_t, Allocator>
    // so one vector with pointers to the initial pointcloud

    // The clusters that are too small or too large in size can also be extracted separately:
    //cec.getRemovedClusters (small_clusters, large_clusters);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "number of clusters after CEC"<<clusters->size());

    /* USED FOR TESING PURPOSES
    // publishes all the clusters (result of CEC) so we can visualize them in Rviz
    //visualize_clusters(clusters, cloud_filtered); 
    */

    

    find_two_max_intensity_clusters(clusters, cloud_filtered);    
    visualize_final_2_clusters(clusters, cloud_filtered);

    
    pcl::PointCloud<pcl::PointXYZI>::Ptr first_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr second_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);

    /*Testing
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "number of max intensityclusters is:"<<final_clusters_indices_to_initial_pointcloud.size());
    */

    // copyPointCloud function to have the 2 clusters as seperate pointclouds
    pcl::copyPointCloud(*cloud_filtered, (*clusters)[final_clusters_indices_to_initial_pointcloud[0]], *first_pointcloud);
    pcl::copyPointCloud(*cloud_filtered, (*clusters)[final_clusters_indices_to_initial_pointcloud[1]], *second_pointcloud);


    // compute centroid point of the two pointclouds
    pcl::computeCentroid(*first_pointcloud, this->first_centroid);
    pcl::computeCentroid(*second_pointcloud, this->second_centroid);
    

    
    // for every cluster-pointcloud measure the distance from the centroid point
    // distance from sensor frame (0,0,0)
    pcl::PointXYZI zero_point(0,0,0);

    this->first_distance = pcl::euclideanDistance(first_centroid, zero_point );
    this->second_distance = pcl::euclideanDistance(second_centroid, zero_point);   
    
    reflectors_distances_msg.distance_to_first_cluster = this->first_distance;
    reflectors_distances_msg.distance_to_second_cluster = this->second_distance;


    RCLCPP_INFO_STREAM(rclcpp::get_logger("first_distance"), this->first_distance);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("second_distance"), this->second_distance);

    // published once, it needs no packet loss!
    reflectors_distances_publisher_->publish(reflectors_distances_msg);

    
    // node no longer needed->shutdown
    rclcpp::shutdown();
}

// wait for "true" msg from topic "/start_parking"
void LidarParkingNode::start_parking_callback(const std_msgs::msg::Bool::SharedPtr msg){

  if ((*msg).data == true){
    
    start_parking = true;

    // messages received only with BestEffort Reliability, so we need to declare "rclcpp::SensorDataQoS()" as the QoS 
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points", rclcpp::SensorDataQoS(), std::bind(&LidarParkingNode::lidar_callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Subscribed to topic \"/points\", waiting for \"PointCloud2\" message..");

  }

}

// finds the indiced of the 2 maximum intensity clusters
// and puts them in the private field final_clusters_indices_to_initial_pointcloud
void LidarParkingNode::find_two_max_intensity_clusters(pcl::IndicesClustersPtr clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered){


  double f = numeric_limits<float>::infinity();
  double negInf= f*-1;

  double first_max_mean_intensity = negInf;
  double second_max_mean_intensity = negInf;
  double temp_mean_intensity;
  double sum_intensities;

  //gia kathe mia apo tis 2 megistes times entasis prepei na ehw apothikeumeno kai to index tou cluster
  int first_max_mean_instensity_cluster_index;
  int second_max_mean_intensity_cluster_index;

  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);

  int i, j;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "number of clusters:"<<clusters->size());


  for(i=0; i<clusters->size(); i++){
    temp_mean_intensity = 0;
    sum_intensities = 0;
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "find_two_maxity_intensity_clusters before copy");

    pcl::copyPointCloud(*cloud_filtered, (*clusters)[i], *temp_pointcloud);

    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "find_two_maxity_intensity_clusters after copy");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "i: "<< i);

    for(j=0; j<temp_pointcloud->points.size(); j++){
      sum_intensities += temp_pointcloud->points[j].intensity;

    }
    temp_mean_intensity = sum_intensities/(double)j;
    //afou tha ehei upologistei to mean value tis entasis tou sugkekrimenou cluster
    //tha to sugkrinoume me to prwto kai to 2o megisto pou ehoume ews twra

  
    if(temp_mean_intensity > first_max_mean_intensity){
      second_max_mean_intensity = first_max_mean_intensity;
      second_max_mean_intensity_cluster_index = first_max_mean_instensity_cluster_index;

      first_max_mean_intensity = temp_mean_intensity;
      first_max_mean_instensity_cluster_index = i;


    }else if(temp_mean_intensity > second_max_mean_intensity){
      second_max_mean_intensity = temp_mean_intensity;
      second_max_mean_intensity_cluster_index = i;
    }
  }

  final_clusters_indices_to_initial_pointcloud.push_back(first_max_mean_instensity_cluster_index);
  final_clusters_indices_to_initial_pointcloud.push_back(second_max_mean_intensity_cluster_index);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "first_max_mean_intensity_cluster_index: "<< first_max_mean_instensity_cluster_index);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "second_max_mean_intensity_cluster_index: "<< second_max_mean_intensity_cluster_index);


  //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "find_two_max_intensity_clusters");
}

// USED FOR TESING PURPOSES 
// Publishes the 2 max intensity clusters(from final_clusters_indices_to_initial_pointcloud field) 
// as seperate PointClouds so we can visualize them in Rviz
void LidarParkingNode::visualize_final_2_clusters(pcl::IndicesClustersPtr clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered){
  //apo ta pedia tis klasis me ta index tha parw ekeina ta upopointcloud kai tha ta prosthesw se ena kai meta publish
  //mipws na to kanw se 2 pointcloud kalutera prwta

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_1 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_2 (new pcl::PointCloud<pcl::PointXYZI>);


  pcl::copyPointCloud(*cloud_filtered, (*clusters)[final_clusters_indices_to_initial_pointcloud[0]], *pcl_cluster_1);
  pcl::copyPointCloud(*cloud_filtered, (*clusters)[final_clusters_indices_to_initial_pointcloud[1]], *pcl_cluster_2);


  pcl::toROSMsg(*pcl_cluster_1, cluster_one_msg);
  pcl::toROSMsg(*pcl_cluster_2, cluster_two_msg);  

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cluster_one_msg.height "<< cluster_one_msg.height);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cluster_one_msg.width "<< cluster_one_msg.width);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cluster_two_msg.height "<< cluster_two_msg.height);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cluster_two_msg.width "<< cluster_two_msg.width);
  

  // publish clusters interchangeably for better visualization
  int i = 0;
  while(rclcpp::ok()){
    if(i%2 == 0){
      cluster_one_publisher_->publish(cluster_one_msg );
    }else{
      cluster_one_publisher_->publish(cluster_two_msg );
    }
    sleep(2);

    i++;
  }


  
}

// USED FOR TESING PURPOSES (CEC)
// publishes all the clusters(result of CEC) as one pointcloud so we can visualize them in Rviz
void LidarParkingNode::visualize_clusters(pcl::IndicesClustersPtr clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered){

    //testing
    //show clusters in rviz
    pcl::PointCloud<pcl::PointXYZI>::Ptr main_pointcloud_result (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);


    
    for(int i=0; i<clusters->size(); i++){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"clusters->size() stin visualize_clusters"<<clusters->size());
      pcl::copyPointCloud(*cloud_filtered, (*clusters)[i], *temp_pointcloud);
      if(i==0){
        *main_pointcloud_result = *temp_pointcloud;
      }else{
        *main_pointcloud_result+=*temp_pointcloud;
      }
        
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "(*main_pointcloud_result).size();"<<(*main_pointcloud_result).size());


    pcl::toROSMsg(*main_pointcloud_result, rclcpp_pointcloud2_result);    

    
    //publish to topic
    while(true){
      testing_pointcloud_publisher_->publish(rclcpp_pointcloud2_result);
      sleep(1);

    }
}
