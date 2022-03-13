#include "../include/camera/logical_camera.h"

LogicalCamera::LogicalCamera(ros::NodeHandle & node) 
: tfBuffer(), tfListener(tfBuffer)
{
    node_ = node;
    
    logical_camera_station1_subscriber = node_.subscribe("/ariac/logical_camera_station1", 10, &LogicalCamera::logical_camera_station1_callback, this);

    logical_camera_station2_subscriber = node_.subscribe("/ariac/logical_camera_station2", 10, &LogicalCamera::logical_camera_station2_callback, this);

    logical_camera_station3_subscriber = node_.subscribe("/ariac/logical_camera_station3", 10, &LogicalCamera::logical_camera_station3_callback, this);

    logical_camera_station4_subscriber = node_.subscribe("/ariac/logical_camera_station4", 10, &LogicalCamera::logical_camera_station4_callback, this);

    quality_control_sensor1_subscriber = node_.subscribe("/ariac/quality_control_sensor_1", 10, &LogicalCamera::quality_control_sensor1_callback, this);

    quality_control_sensor2_subscriber = node_.subscribe("/ariac/quality_control_sensor_2", 10, &LogicalCamera::quality_control_sensor2_callback, this);

    quality_control_sensor3_subscriber = node_.subscribe("/ariac/quality_control_sensor_3", 10, &LogicalCamera::quality_control_sensor3_callback, this);

    quality_control_sensor4_subscriber = node_.subscribe("/ariac/quality_control_sensor_4", 10, &LogicalCamera::quality_control_sensor4_callback, this);

}

void LogicalCamera::logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
     ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins0: '" << image_msg->models.size() << "' objects.");
  
     if (get_bins0)
     { 

      // tf2_ros::Buffer tfBuffer;
      // tf2_ros::TransformListener tfListener(tfBuffer);

      //  ros::Rate rate(10);
      ros::Duration timeout(5.0);

      unsigned short int i{0};
      unsigned short int part_index{1}; 
      while(i < image_msg->models.size())
      {
  
        std::string product_type = image_msg->models.at(i).type;
        if(i!= 0 && image_msg->models.at(i).type != image_msg->models.at(i-1).type){
          part_index = 1;
        }
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.id = std::to_string(part_index);
        
        std::string frame_name = "logical_camera_bins0_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
        product.frame = frame_name;        
        
        geometry_msgs::TransformStamped transformStamped;

        transformStamped = tfBuffer.lookupTransform("world", frame_name, ros::Time(0), timeout);
        
        product.time_stamp = ros::Time(0);
        product.world_pose.position.x = transformStamped.transform.translation.x;
        product.world_pose.position.y = transformStamped.transform.translation.y;
        product.world_pose.position.z = transformStamped.transform.translation.z;

        product.world_pose.orientation.x = transformStamped.transform.rotation.x;
        product.world_pose.orientation.y = transformStamped.transform.rotation.y;
        product.world_pose.orientation.z = transformStamped.transform.rotation.z;
        product.world_pose.orientation.w = transformStamped.transform.rotation.w; 

        product_list0_.push_back(product);
        camera_parts_list.at(0).push_back(product);

        // ROS_INFO_STREAM("shera :" << product_list0_.at(i).frame);

        // if (order_list_.at(0).kitting.at(0).shipment_type == product.type){
        //  find_part(product.type);
        // ROS_INFO_STREAM("hai" << i);
        // }

        i++;
        part_index++; 
      }
      
     get_bins0 = false; 
     }

}

std::array<std::vector<Product>,2> LogicalCamera::findparts(){
  ros::Subscriber logical_camera_bins0_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins0", 10, 
    &LogicalCamera::logical_camera_bins0_callback, this);

  ros::Subscriber logical_camera_bins1_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins1", 10, 
    &LogicalCamera::logical_camera_bins1_callback, this);

  ros::Duration(0.5).sleep();
  return camera_parts_list;  

}


std::vector<Product> LogicalCamera::get_product_list0(){
  return product_list0_;
}


void LogicalCamera::logical_camera_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
     ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins1: '" << image_msg->models.size() << "' objects.");
    
    if (get_bins1)
     { 

      // tf2_ros::Buffer tfBuffer;
      // tf2_ros::TransformListener tfListener(tfBuffer);

      //  ros::Rate rate(10);
      ros::Duration timeout(5.0);

      unsigned short int i{0};
      unsigned short int part_index{1}; 
      while(i < image_msg->models.size())
      {
  
        std::string product_type = image_msg->models.at(i).type;
        if(i!= 0 && image_msg->models.at(i).type != image_msg->models.at(i-1).type){
          part_index = 1;
        }
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.id = std::to_string(part_index);
        
        std::string frame_name = "logical_camera_bins1_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
        product.frame = frame_name;        
        
        geometry_msgs::TransformStamped transformStamped;

        transformStamped = tfBuffer.lookupTransform("world", frame_name, ros::Time(0), timeout);
        
        product.time_stamp = ros::Time(0);
        product.world_pose.position.x = transformStamped.transform.translation.x;
        product.world_pose.position.y = transformStamped.transform.translation.y;
        product.world_pose.position.z = transformStamped.transform.translation.z;

        product.world_pose.orientation.x = transformStamped.transform.rotation.x;
        product.world_pose.orientation.y = transformStamped.transform.rotation.y;
        product.world_pose.orientation.z = transformStamped.transform.rotation.z;
        product.world_pose.orientation.w = transformStamped.transform.rotation.w; 

        product_list1_.push_back(product);
        camera_parts_list.at(1).push_back(product);

        // if (order_list_.at(0).kitting.at(0).shipment_type == product.type){
        //  find_part(product.type);
        // ROS_INFO_STREAM("hai" << i);
        // }

        i++;
        part_index++; 
      }
      
     get_bins1 = false; 
     }

}

void LogicalCamera::logical_camera_station1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 1: '" << image_msg->models.size() << "' objects.");

}   

void LogicalCamera::logical_camera_station2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 2: '" << image_msg->models.size() << "' objects.");

}

void LogicalCamera::logical_camera_station3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 3: '" << image_msg->models.size() << "' objects.");

}

void LogicalCamera::logical_camera_station4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 4: '" << image_msg->models.size() << "' objects.");

}

void LogicalCamera::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv1");
  }

}

void LogicalCamera::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv2");
  }
}

void LogicalCamera::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv3");
  }
}

void LogicalCamera::quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on 4");
  }
}


// SENSOR BLACKOUT
// std::string LogicalCamera::CheckBlackout(){
//   flag1 = "yes";
//   flag2 = "yes";

//   quality_control_sensor1_subscriber_b1 = node_.subscribe("/ariac/quality_control_sensor_1", 2, &LogicalCamera::callback_b1, this);

//   quality_control_sensor2_subscriber_b2 = node_.subscribe("/ariac/quality_control_sensor_2", 2, &LogicalCamera::callback_b2, this);

//   if(flag1 == "no" && flag2 == "no"){
//     return "no";
//   }
//   else{
//     return "yes";
//   }
//   }

// void LogicalCamera::callback_b1(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
//   flag1 = "no";
// }

// void LogicalCamera::callback_b2(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
//   flag2 = "no";
// }