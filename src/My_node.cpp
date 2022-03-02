/**
 * @file try.cpp
 * @author Pulkit Mehta (pmehta09@umd.edu)
 * @brief Node for RWA1
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <vector>

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
#include <nist_gear/DetectedProduct.h>
#include <nist_gear/SubmitShipment.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/AssemblyStationSubmitShipment.h>


#include "../include/comp/comp_class.h"
#include "../include/agv/agv.h"
#include "../include/util/util.h"
/**
 * @brief Start the competition by waiting for and then calling the start ROS Service.
 * 
 * @param node Nodehandle
 */
void start_competition(ros::NodeHandle & node)
{
  // create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // if it's not already ready, wait for it to be ready.
  // calling the Service using the client before the server is ready would fail.
  if (!start_client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // combination of the "request" and the "response".
  start_client.call(srv);  // call the start Service.
  // if not successful, print out why.
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("Competition started!");
  }
}
 
void end_competition(ros::NodeHandle & node)
{
  ros::ServiceClient end_client = node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("Competition ended!");
  }
  ros::shutdown();
}

void as_submit_assembly(ros::NodeHandle & node, std::string s_id, std::string st)
{
  ros::ServiceClient client1 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as1/submit_shipment");
  ros::ServiceClient client2 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as2/submit_shipment");
  ros::ServiceClient client3 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as3/submit_shipment");
  ros::ServiceClient client4 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as4/submit_shipment");
   
  nist_gear::AssemblyStationSubmitShipment srv;
  srv.request.shipment_type = st;

  if(s_id == "as1"){
    client1.call(srv);
    }
  if(s_id == "as2"){
    client2.call(srv);
    }
}


int main(int argc, char ** argv)
{
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);


  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);

  
  // Subscribe to the '/ariac/logical_camera_bins8' topic.
  ros::Subscriber logical_camera_bins8_subscriber = node.subscribe(
    "/ariac/logical_camera_station1", 10,
    &MyCompetitionClass::logical_camera_bins0_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_station2' topic.
  ros::Subscriber logical_camera_station2_subscriber = node.subscribe(
    "/ariac/logical_camera_station2", 10,
    &MyCompetitionClass::logical_camera_station2_callback, &comp_class);

  // Subscribe to the '/ariac/quality_control_sensor_1' topic.
  ros::Subscriber quality_control_sensor1_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_1", 10,
    &MyCompetitionClass::quality_control_sensor1_callback, &comp_class);

  // Subscribe to the '/ariac/quality_control_sensor_2' topic.
  ros::Subscriber quality_control_sensor2_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_2", 10,
    &MyCompetitionClass::quality_control_sensor2_callback, &comp_class);

  // Subscribe to the '/ariac/quality_control_sensor_3' topic.
  ros::Subscriber quality_control_sensor3_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_3", 10,
    &MyCompetitionClass::quality_control_sensor3_callback, &comp_class);

  // Subscribe to the '/ariac/quality_control_sensor_4' topic.
  ros::Subscriber quality_control_sensor4_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_4", 10,
    &MyCompetitionClass::quality_control_sensor4_callback, &comp_class);   

  // Subscribe to the '/ariac/agv1/station' topic.
  ros::Subscriber agv1_station_subscriber = node.subscribe(
    "/ariac/agv1/station", 10,
    &MyCompetitionClass::agv1_station_callback, &comp_class);

  // Subscribe to the '/ariac/agv2/station' topic.
  ros::Subscriber agv2_station_subscriber = node.subscribe(
    "/ariac/agv2/station", 10,
    &MyCompetitionClass::agv2_station_callback, &comp_class);

  // Subscribe to the '/ariac/agv3/station' topic.
  ros::Subscriber agv3_station_subscriber = node.subscribe(
    "/ariac/agv3/station", 10,
    &MyCompetitionClass::agv3_station_callback, &comp_class);

  // Subscribe to the '/ariac/agv4/station' topic.
  ros::Subscriber agv4_station_subscriber = node.subscribe(
    "/ariac/agv4/station", 10,
    &MyCompetitionClass::agv4_station_callback, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);
  Agv agv(node);
  // std::string state = comp_class.getCompetitionState();


  // ros::Duration(10);
  std::vector<Order> orders;
  std::vector<Kitting> kittings;
  std::string agv_id;
  std::string kshipment_type;
  std::string ashipment_type;
  std::string kstation_id;
  std::string astation_id;

  int counter = 0;
  
  while(ros::ok())
  {
    orders = comp_class.get_order_list();
    if(orders.size() !=0 && counter!=1)
    {
      agv_id = comp_class.get_agv_id();
      kstation_id = orders.at(0).kitting.at(0).station_id;
      kshipment_type = orders.at(0).kitting.at(0).shipment_type;
      ashipment_type = orders.at(0).assembly.at(0).shipment_type;

      agv.submit_shipment(agv_id,kshipment_type,kstation_id);
      if (agv.get_agv2_station() == kstation_id){
        as_submit_assembly(node, astation_id, ashipment_type);
      }

      
      

      // ROS_INFO_STREAM("AGV ID : " << agv_id);
      counter++;
    }
    ros::spinOnce();
  }



  // ROS_INFO_STREAM("Current order is: " << orders.at(0).order_id);
  
  // Order cur_order = orders.at(0);
  // Kitting cur_kit = cur_order.kitting.at(0);
  // agv.submit_shipment(cur_kit.agv_id);
  // std::cout << cur_kit.agv_id << '\n';  

   // This executes callbacks on new data until ctrl-c.

  return 0;
}