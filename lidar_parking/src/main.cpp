#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "LidarParkingNode.h"

int main(int argc, char *argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarParkingNode>());
    rclcpp::shutdown();

    return 0;
}
