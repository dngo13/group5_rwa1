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
#include "../include/arm/arm.h"


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

  // create an instance of the kitting arm
  motioncontrol::Arm arm(node);
  arm.init();

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
  Product remaining_part;
  std::vector<std::string> parts_not_found;
  unsigned short int cur_order_index{0};
  comp_state = comp_class.getCompetitionState();
  auto competition_start_time = comp_class.getClock();


  ros::ServiceClient client1 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as1/submit_shipment");
  ros::ServiceClient client2 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as2/submit_shipment");
  ros::ServiceClient client3 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as3/submit_shipment");
  ros::ServiceClient client4 = node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/as4/submit_shipment");
   
  nist_gear::AssemblyStationSubmitShipment asrv;

  bool lookonce = true;
  bool order0_models_found = false;
  bool order1_models_found = false;
  bool not_found = false;
  bool is_insufficient = false;
  bool blackout = true;
  ros::Time time;

  ros::Rate rate = 2;	  
  rate.sleep();	

  auto list = cam.findparts();
  // auto camera_bins0_data = list.at(0);
  // auto camera_bins1_data = list.at(1);
  
  cam.segregate_parts(list);
  auto cam_map = cam.get_camera_map();


  while(ros::ok){
  
  orders = comp_class.get_order_list();
  
  arm.goToPresetLocation("home1");
  arm.goToPresetLocation("home2");
  

  if (!order0_models_found){
    //kitting
    kittings = orders.at(0).kitting;
    for(auto &kit: orders.at(0).kitting){
      
      kshipment_type = kit.shipment_type;
      agv_id = kit.agv_id;
      products = kit.products;

      
      // auto p = cam_map.find(kit.products.at(0).type);
      // ROS_INFO_STREAM(p->second.at(0).frame);
      
      // if (p != cam_map.end()){
        
      // }
      // std::vector<std::pair<Product, std::string> > camera_for_product{};
      std::vector<Product> parts_for_kitting;
      // // products
      // for (auto &part:kit.products){
      //   // ROS_INFO_STREAM("Part type: " << product.type);
      //   for (auto &camlist: list){
      //     int it{0};
      //     for (auto &campart: camlist){
      //       if(campart.type == part.type){
      //         if(parts_for_kitting.size() == kit.products.size()){
      //         break;
      //         }
      //         campart.target_pose = part.frame_pose;
      //         parts_for_kitting.push_back(campart);
      //         // ROS_INFO_STREAM("campart: " << campart.type);
      //         camlist.erase(camlist.begin() + it);
      //         break;
      //       }
      //       it++;
      //     }
      //   }
      // }

      for (auto &part:kit.products){
        // parts_for_kitting.push_back(part);
        parts_for_kitting.push_back(part);
        // auto p = cam_map.find(part.type);
        
        // ROS_INFO_STREAM(p->second.at(0).frame_pose);

      }
      

      for(auto &iter: parts_for_kitting){
        if(comp_class.high_priority_announced){
          remaining_part = iter;
          order1_models_found = true;
          break;
        }
        auto p = cam_map.find(iter.type);
        for (int i{0}; i < p->second.size(); i++){
          if(p->second.at(i).status.compare("free") == 0){
            arm.movePart(iter.type, p->second.at(i).world_pose, iter.frame_pose, kit.agv_id);
            // p->second.at(i).status = "processed";
            cam_map[iter.type].at(i).status = "processed";
            
            auto faulty_list = cam.get_faulty_part_list();
            double outside_time = ros::Time::now().toSec();
            double inside_time = ros::Time::now().toSec();
            ROS_INFO_STREAM("hello");
            while (inside_time - outside_time < 5.0) {
                inside_time = ros::Time::now().toSec();
            }
            ROS_INFO_STREAM("again");
            
            if(cam.isFaulty){
              ROS_INFO_STREAM("ready to add");
              auto world_pose = motioncontrol::transformtoWorldFrame(iter.frame_pose,kit.agv_id);
              arm.pickPart(iter.type, world_pose);
              arm.goToPresetLocation("home2");
              arm.deactivateGripper();
              cam.isFaulty = false;
              continue;
            }

            break;
          } 
        }
        

      }  
    }
    order0_models_found = true;
  }
  
  // if(abs(ros::Time::now().toSec() - cam.CheckBlackout())>5){
  //       ROS_INFO_STREAM("Sensor_blackout");
  // }

 
  
  ROS_INFO_STREAM(orders.size());

  if (orders.size() > 1){
    if(order1_models_found){
    //kittings
      for(auto &kit: orders.at(1).kitting){
        
        kshipment_type = kit.shipment_type;
        ROS_INFO_STREAM(kshipment_type);

        std::vector<Product> parts_for_kitting;

        for (auto &part:kit.products){
          parts_for_kitting.push_back(part);
        }
        
        for(auto &iter: parts_for_kitting){
          auto p = cam_map.find(iter.type);
          for (int i{0}; i < p->second.size(); i++){
            if(p->second.at(i).status.compare("free") == 0){
              arm.movePart(iter.type, p->second.at(i).world_pose, iter.frame_pose, kit.agv_id);
              p->second.at(i).status = "processed";
              break;
            } 
          }
          

        }  
      }
    }
    
    order1_models_found = false;
  }

  }
  
  ros::waitForShutdown();  
}
