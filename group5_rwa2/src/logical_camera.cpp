#include "../include/camera/logical_camera.h"

LogicalCamera::LogicalCamera(ros::NodeHandle & node){

    logical_camera_bins0_subscriber = node.subscribe("/ariac/logical_camera_bins0", 10, &LogicalCamera::logical_camera_bins0_callback, this);

    logical_camera_bins1_subscriber = node.subscribe("/ariac/logical_camera_bins1", 10, &LogicalCamera::logical_camera_bins1_callback, this);

    logical_camera_station1_subscriber = node.subscribe("/ariac/logical_camera_station1", 10, &LogicalCamera::logical_camera_station1_callback, this);

    logical_camera_station2_subscriber = node.subscribe("/ariac/logical_camera_station2", 10, &LogicalCamera::logical_camera_station2_callback, this);

    logical_camera_station3_subscriber = node.subscribe("/ariac/logical_camera_station3", 10, &LogicalCamera::logical_camera_station3_callback, this);

    logical_camera_station4_subscriber = node.subscribe("/ariac/logical_camera_station4", 10, &LogicalCamera::logical_camera_station4_callback, this);

    quality_control_sensor1_subscriber = node.subscribe("/ariac/quality_control_sensor_1", 10, &LogicalCamera::quality_control_sensor1_callback, this);

    quality_control_sensor2_subscriber = node.subscribe("/ariac/quality_control_sensor_2", 10, &LogicalCamera::quality_control_sensor2_callback, this);

    quality_control_sensor3_subscriber = node.subscribe("/ariac/quality_control_sensor_3", 10, &LogicalCamera::quality_control_sensor3_callback, this);

    quality_control_sensor4_subscriber = node.subscribe("/ariac/quality_control_sensor_4", 10, &LogicalCamera::quality_control_sensor4_callback, this);

}

void LogicalCamera::logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
     ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins0: '" << image_msg->models.size() << "' objects.");
    
    //  for (const auto &model: image_msg->models){
    //    Product product;
    //    product.type = model.type; 
    //    product.frame_pose = model.pose; 
    //    product.frame = "logical_camera_bins0_" + model.type + "_frame";
    //    product_list_.push_back(product);
    //  }


     if (get_bins0)
     { 
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);

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
        // ROS_INFO_STREAM(product_list0_.at(i).world_pose.position.x);

        i++;
        part_index++;

        
      }

     get_bins0 = false; 
     }
}


std::vector<Product> LogicalCamera::get_product_list0(){
  return product_list0_;
}




void LogicalCamera::logical_camera_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
     ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins1: '" << image_msg->models.size() << "' objects.");
    //  ROS_INFO_STREAM(image_msg->models.at(0).type);
     
    //  for (const auto &model: image_msg->models){
    //   for(int i=0; i< image_msg->models.size(); i++){
    //   //  ROS_INFO("hello");
    //   // ROS_INFO_STREAM(image_msg->models.at(i).type);

    //    Product product;
    //    product.type = image_msg->models.at(i).type; 
    //    product.frame_pose = image_msg->models.at(i).pose; 
    //    product.frame = "logical_camera_bins0_" + image_msg->models.at(i).type + "_frame";
    //    product_list_.push_back(product);
    //   //  ROS_INFO("part: ", product.type); 
    //     // ROS_INFO_STREAM(product.type);

    //  } 

}


std::vector<Product> LogicalCamera::get_product_list1(){
  return product_list1_;
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
