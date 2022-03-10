/**
 * @file My_node.cpp
 * @author Pulkit Mehta (pmehta09@umd.edu)
 * @author Darshan Jain (djain12@umd.edu)
 * @author Jeffin J K (jeffinjk@umd.edu)
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
  ros::init(argc, argv, "My_node");

  ros::NodeHandle node;
  // ros::AsyncSpinner spinner(2);
  // spinner.start();

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);
  comp_class.init();

  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_bins8' topic.
  ros::Subscriber logical_camera_bins0_subscriber = node.subscribe(
    "/ariac/logical_camera_bins0", 10,
    &MyCompetitionClass::logical_camera_bins0_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_station2' topic.
  ros::Subscriber logical_camera_station2_subscriber = node.subscribe(
    "/ariac/logical_camera_station2", 10,
    &MyCompetitionClass::logical_camera_station2_callback, &comp_class);

  // ros::Subscriber depth_camera_bins1_subscriber = node.subscribe(
  //   "/ariac/depth_camera_bins1/depth/image_raw --noarr", 10,
  //   &MyCompetitionClass::depth_camera_bins1_callback, &comp_class);

  ros::Subscriber proximity_sensor_subscriber = node.subscribe(
    "/ariac/proximity_sensor_0", 10,
     &MyCompetitionClass::proximity_sensor0_callback,&comp_class);

  ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/breakbeam_0", 10,
    &MyCompetitionClass::breakbeam0_callback, &comp_class);

  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_0", 10,
    &MyCompetitionClass::laser_profiler0_callback,&comp_class);
  
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

  ROS_INFO("Setup complete.");
  
  Agv agv(node);

  // ros::Duration(10).sleep();
  std::vector<Order> orders;
  std::vector<Kitting> kittings;
  std::string agv_id;
  std::string kshipment_type;
  std::string ashipment_type;
  std::string kstation_id;
  std::string astation_id;
  std::string comp_state;
  unsigned short int cur_order_index{0};
  
  ros::ServiceClient client1 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as1/submit_shipment");
  ros::ServiceClient client2 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as2/submit_shipment");
  ros::ServiceClient client3 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as3/submit_shipment");
  ros::ServiceClient client4 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as4/submit_shipment");
   
  nist_gear::AssemblyStationSubmitShipment asrv;

  bool ship1 = false;
  bool ship2 = false;
  ros::Timer timer = node.createTimer(ros::Duration(1), &MyCompetitionClass::callback, &comp_class);

  while(ros::ok())
  {
  // do{
  //   orders = comp_class.get_order_list();
  // }while (orders.size() == 0);

    orders = comp_class.get_order_list();
    comp_state = comp_class.getCompetitionState();
    auto competition_start_time = comp_class.getClock();
  
    if(orders.size() !=0 && comp_class.get_timer())
    // if(comp_class.get_timer())
    
    {
      // && !orders.at(cur_order_index).order_processed Do this inside this loop as it accesses it before asssignment otherwise.
      // if (!orders.at(cur_order_index).order_processed){
        agv_id = comp_class.get_agv_id();
        kstation_id = orders.at(cur_order_index).kitting.at(cur_order_index).station_id;
        kshipment_type = orders.at(cur_order_index).kitting.at(cur_order_index).shipment_type;
        ashipment_type = orders.at(cur_order_index).assembly.at(cur_order_index).shipment_type;
        astation_id = orders.at(cur_order_index).assembly.at(cur_order_index).stations;

        asrv.request.shipment_type = ashipment_type;

        if(!ship1){
          agv.shipAGV(agv_id,kshipment_type,kstation_id);
          ship1 = true;

        } 

        if (agv.get_agv1_station() == kstation_id && !ship2){
          as_submit_assembly(node, astation_id, ashipment_type);
          ship2 = true; 
        }
        if (agv.get_agv2_station() == kstation_id && !ship2){
          as_submit_assembly(node, astation_id, ashipment_type);
          ship2 = true; 
        }
        if (agv.get_agv3_station() == kstation_id && !ship2){
          as_submit_assembly(node, astation_id, ashipment_type);
          ship2 = true; 
        }
        if (agv.get_agv4_station() == kstation_id && !ship2){
          as_submit_assembly(node, astation_id, ashipment_type);
          ship2 = true; 
        }

        // orders.at(cur_order_index).order_processed = true;

        if(comp_state == "done"){
          comp_class.endCompetition();
        }

        // cur_order_index += 1;
      // }

    }
    
    ros::spinOnce();
    } 
  // ros::waitForShutdown();
  return 0;
  
}
