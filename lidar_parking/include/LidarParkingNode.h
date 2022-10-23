//ROS
#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
//#include "aboat_interfaces/msg/reflectors_distances.hpp"
#include "clustering_interfaces/msg/distance_to_clusters.hpp"

//PCL
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>


using namespace std;

class LidarParkingNode:public rclcpp::Node{
    private:

        // ROS2 subscriber and related topic name
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_parking_;
        void start_parking_callback(const std_msgs::msg::Bool::SharedPtr msg);

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr testing_pointcloud_publisher_;
        sensor_msgs::msg::PointCloud2 rclcpp_pointcloud2_result;

        

        //testing purposes visualization
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_one_publisher_;
        sensor_msgs::msg::PointCloud2 cluster_one_msg; 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_two_publisher_;
        sensor_msgs::msg::PointCloud2 cluster_two_msg;


        rclcpp::Publisher<clustering_interfaces::msg::DistanceToClusters>::SharedPtr reflectors_distances_publisher_;
        clustering_interfaces::msg::DistanceToClusters reflectors_distances_msg;

        double first_distance = 0;
        double second_distance = 0;

        pcl::PointXYZI first_centroid;
        pcl::PointXYZI second_centroid;

        vector<int> final_clusters_indices_to_initial_pointcloud;

        bool start_parking = false;


    public:
        LidarParkingNode();
        void find_two_max_intensity_clusters(pcl::IndicesClustersPtr clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered); 
        
        void visualize_clusters(pcl::IndicesClustersPtr clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered);
        void visualize_final_2_clusters(pcl::IndicesClustersPtr clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered);

};