/**
 * @file My_node.cpp
 * @author Pulkit Mehta (pmehta09@umd.edu)
 * @author Darshan Jain (djain12@umd.edu)
 * @author Jeffin J K (jeffinjk@umd.edu)
 * @brief Node for RWA2
 * @version 0.1
 * @date 2022-03-05
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
#include "../include/camera/logical_camera.h"


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
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);
  comp_class.init();

  LogicalCamera cam(node);

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
  
  ROS_INFO("Setup complete.");
  
  Agv agv(node);

  std::vector<Order> orders;
  std::vector<Kitting> kittings;
  std::vector<Product> products;
  std::string agv_id;
  std::string kshipment_type;
  std::string ashipment_type;
  std::string kstation_id;
  std::string astation_id;
  std::string comp_state;
  Product product;
  std::vector<std::string> parts_not_found;
  unsigned short int cur_order_index{0};
  comp_state = comp_class.getCompetitionState();
  auto competition_start_time = comp_class.getClock();


  ros::ServiceClient client1 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as1/submit_shipment");
  ros::ServiceClient client2 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as2/submit_shipment");
  ros::ServiceClient client3 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as3/submit_shipment");
  ros::ServiceClient client4 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as4/submit_shipment");
   
  nist_gear::AssemblyStationSubmitShipment asrv;

 
  bool order0_models_found = false;
  bool order1_models_found = false;
  bool not_found = false;
  bool is_insufficient = false;
  bool blackout = true;
  ros::Time time;

  ros::Rate rate = 2;	  
  rate.sleep();	
  while(ros::ok){

  orders = comp_class.get_order_list();


  if (!order0_models_found){
    kittings = orders.at(0).kitting;
    for(auto &kit: orders.at(0).kitting){
      kshipment_type = kit.shipment_type;
      agv_id = kit.agv_id;
      products = kit.products;
      for (auto &part:kit.products){
        product = part;
        ROS_INFO_STREAM("Part type: " << product.type);
        bool flag1 = false;
        bool flag2 = true;

        // while(flag2){
        //   auto list = cam.findparts();
        //   for (auto logcam: list){
        //     if(logcam.empty() == true){
        //     }
        //     for (auto model: logcam){
        //       if(product.type == model.type){
        //         ROS_INFO_STREAM("frame: " << '\n' << model.frame);
        //       }
        //     }
        //   }
        //   flag2 = false;
        // }
      }
    }
    order0_models_found = true;
  }

  // if (orders.size() > 1 && !order1_models_found){
  //   kittings = orders.at(1).kitting;
  //   for(auto &kit: orders.at(1).kitting){
  //     kshipment_type = kit.shipment_type;
  //     agv_id = kit.agv_id;
  //     products = kit.products;
  //     for (auto &part:kit.products){
  //       product = part;
  //       ROS_INFO_STREAM("Part type: " << product.type);
  //       bool flag1 = false;
  //       bool flag2 = true;

  //       while(flag2){
  //         auto list = cam.findparts();
  //         for (auto logcam: list){
  //           if(logcam.empty() == true){
  //           }
  //           for (auto model: logcam){
  //             if(product.type == model.type){
  //                ROS_INFO_STREAM("frame: " << '\n' << model.frame);              }
  //             else{
  //               parts_not_found.push_back(model.type);            
  //             }
  //           }
  //         }
  //         flag2 = false;
  //       }
  //     }
  //   }
  //   order1_models_found = true;
  // }

  }
  
  ros::waitForShutdown();  
}
