#ifndef UTILS_H
#define UTILS_H

#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h> 
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/AssemblyStationSubmitShipment.h>

typedef struct Product
{
    std::string type;
    geometry_msgs::Pose frame_pose;
}
product;



/**
 * @brief Structure of a kitting shipment
 * 
 */
typedef struct Kitting
{
    std::string shipment_type;
    std::string agv_id;
    std::string station_id;
    std::vector<Product> products;
}
kitting;

/**
 * @brief Structure of an assembly shipment
 * 
 */
typedef struct Assembly
{
    std::string shipment_type;
    std::string stations;
    std::vector<Product> products;
}
assembly;

/**
 * @brief Structure of an order
 * 
 */
typedef struct Order
{   std::string order_id;
    unsigned short int priority;
    unsigned short int kitting_robot_health;
    unsigned short int assembly_robot_health;
    std::string announcement_condition;
    double announcement_condition_value;
    std::vector<Kitting> kitting;
    std::vector<Assembly> assembly;
}
order;

#endif