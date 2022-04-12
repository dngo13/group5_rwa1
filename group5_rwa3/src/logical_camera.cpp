#include "../include/camera/logical_camera.h"

LogicalCamera::LogicalCamera(ros::NodeHandle & node) 
: tfBuffer(), tfListener(tfBuffer)
{
    node_ = node;
    
    // quality_control_sensor1_subscriber = node_.subscribe("/ariac/quality_control_sensor_1", 10, &LogicalCamera::quality_control_sensor1_callback, this);

    // quality_control_sensor2_subscriber = node_.subscribe("/ariac/quality_control_sensor_2", 10, &LogicalCamera::quality_control_sensor2_callback, this);

    // quality_control_sensor3_subscriber = node_.subscribe("/ariac/quality_control_sensor_3", 10, &LogicalCamera::quality_control_sensor3_callback, this);

    // quality_control_sensor4_subscriber = node_.subscribe("/ariac/quality_control_sensor_4", 10, &LogicalCamera::quality_control_sensor4_callback, this);
    
    ros::Subscriber logical_camera_bins0_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins0", 1, 
    &LogicalCamera::logical_camera_bins0_callback, this);

    ros::Subscriber logical_camera_bins1_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins1", 1, 
    &LogicalCamera::logical_camera_bins1_callback, this);

    // timer = node_.createTimer(ros::Duration(2), &LogicalCamera::callback, this);


}

void LogicalCamera::callback(const ros::TimerEvent& event){
  wait = false;
}

bool LogicalCamera::get_timer(){
  return wait;
}


void LogicalCamera::logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    //  ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins0: '" << image_msg->models.size() << "' objects.");
    //  if(image_msg->models.empty()){
    //    logflag1 = false;
    //  } 

     blackout_time_ = ros::Time::now().toSec(); 
     if (get_cam[0])
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
        if(i == 0 && image_msg->models.at(i).type.compare("assembly_battery_blue") == 0){
          part_index = 5;
        }
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_bins0";
        product.id = std::to_string(part_index);
        product.status = "free";
        
        std::string frame_name = "logical_camera_bins0_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
        product.frame = frame_name;        
        // ROS_INFO_STREAM(product.frame);  
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

        i++;
        part_index++; 
      }
      
     get_cam[0] = false; 
     }

}

std::array<std::vector<Product>,19> LogicalCamera::findparts(){
  ros::Subscriber logical_camera_bins0_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins0", 1, 
    &LogicalCamera::logical_camera_bins0_callback, this);

  ros::Subscriber logical_camera_bins1_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins1", 1, 
    &LogicalCamera::logical_camera_bins1_callback, this);

  ros::Subscriber logical_camera_station1_subscriber = node_.subscribe(
    "/ariac/logical_camera_station1", 10, 
    &LogicalCamera::logical_camera_station1_callback, this);

  ros::Subscriber logical_camera_station2_subscriber = node_.subscribe(
    "/ariac/logical_camera_station2", 10, 
    &LogicalCamera::logical_camera_station2_callback, this);

  ros::Subscriber logical_camera_station3_subscriber = node_.subscribe(
    "/ariac/logical_camera_station3", 10, 
    &LogicalCamera::logical_camera_station3_callback, this);

  ros::Subscriber logical_camera_station4_subscriber = node_.subscribe(
    "/ariac/logical_camera_station4", 10, 
    &LogicalCamera::logical_camera_station4_callback, this);

  ros::Subscriber logical_camera_agv1as1_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv1as1", 10, 
    &LogicalCamera::logical_camera_agv1as1_callback, this);

  ros::Subscriber logical_camera_agv1as2_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv1as2", 10, 
    &LogicalCamera::logical_camera_agv1as2_callback, this);

  ros::Subscriber logical_camera_agv1ks_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv1ks", 10, 
    &LogicalCamera::logical_camera_agv1ks_callback, this);

  ros::Subscriber logical_camera_agv2as1_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv2as1", 10, 
    &LogicalCamera::logical_camera_agv2as1_callback, this);

  ros::Subscriber logical_camera_agv2as2_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv2as2", 10, 
    &LogicalCamera::logical_camera_agv2as2_callback, this);

  ros::Subscriber logical_camera_agv2ks_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv2ks", 10, 
    &LogicalCamera::logical_camera_agv2ks_callback, this);

  ros::Subscriber logical_camera_agv3as3_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv3as3", 10, 
    &LogicalCamera::logical_camera_agv3as3_callback, this);

  ros::Subscriber logical_camera_agv3as4_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv3as4", 10, 
    &LogicalCamera::logical_camera_agv3as4_callback, this);

  ros::Subscriber logical_camera_agv3ks_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv3ks", 10, 
    &LogicalCamera::logical_camera_agv3ks_callback, this);

  ros::Subscriber logical_camera_agv4as3_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv4as3", 10, 
    &LogicalCamera::logical_camera_agv4as3_callback, this);

  ros::Subscriber logical_camera_agv4as4_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv4as4", 10, 
    &LogicalCamera::logical_camera_agv4as4_callback, this);

  ros::Subscriber logical_camera_agv4ks_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv4ks", 10, 
    &LogicalCamera::logical_camera_agv4ks_callback, this);

  ros::Subscriber logical_camera_belt_subscriber = node_.subscribe(
    "/ariac/logical_camera_belt", 10, 
    &LogicalCamera::logical_camera_belt_callback, this);

  ros::Duration(0.5).sleep();
  
  return camera_parts_list;  

}

void LogicalCamera::segregate_parts(std::array<std::vector<Product>,19> list){
  for (auto &l: list){
    for(auto &part: l){
      camera_map_[part.type].push_back(part);
    }
  }
}

std::vector<Product> LogicalCamera::get_product_list0(){
  return product_list0_;
}


void LogicalCamera::logical_camera_bins1_callback(
  const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    //  ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins1: '" << image_msg->models.size() << "' objects.");
    // if(image_msg->models.empty()){
    //    logflag2 = false;
    //  } 
    blackout_time_ = ros::Time::now().toSec();
    if (get_cam[1])
     { 
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
        product.camera = "logical_camera_bins1";
        product.id = std::to_string(part_index);
        product.status = "free";

        std::string frame_name = "logical_camera_bins1_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
        product.frame = frame_name;  
        // ROS_INFO_STREAM(product.frame);      
        
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

        i++;
        part_index++; 
      }
      
     get_cam[1] = false; 
     }

}

double LogicalCamera::CheckBlackout(){
  return blackout_time_;
}


void LogicalCamera::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[0]){
      if (!image_msg->models.empty()){
      ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv1");
      isFaulty = true;
      }

      get_faulty_cam[0] = false; 

    }
}


void LogicalCamera::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[1]){
      if (!image_msg->models.empty()){
      ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv2");
      isFaulty = true;
    }
    get_faulty_cam[1] = false;
  }
}


void LogicalCamera::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[2]){
      if (!image_msg->models.empty()){
      ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv3");
      isFaulty = true;
      }
      get_faulty_cam[2] = false;
    }
}


void LogicalCamera::quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[3]){
      if (!image_msg->models.empty()){
      ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on 4");
      isFaulty = true;
      }
    get_faulty_cam[3] = false;
    }
}

std::vector<Product> LogicalCamera::get_faulty_part_list(){
 
  quality_control_sensor1_subscriber = node_.subscribe("/ariac/quality_control_sensor_1", 1, &LogicalCamera::quality_control_sensor1_callback, this);

  quality_control_sensor2_subscriber = node_.subscribe("/ariac/quality_control_sensor_2", 1, &LogicalCamera::quality_control_sensor2_callback, this);

  quality_control_sensor3_subscriber = node_.subscribe("/ariac/quality_control_sensor_3", 1, &LogicalCamera::quality_control_sensor3_callback, this);

  quality_control_sensor4_subscriber = node_.subscribe("/ariac/quality_control_sensor_4", 1, &LogicalCamera::quality_control_sensor4_callback, this);
    
  return faulty_part_list_;
}


void LogicalCamera::logical_camera_station1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 1: '" << image_msg->models.size() << "' objects.");
    if (get_cam[2])
     { 
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
        
        std::string frame_name = "logical_camera_station1_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(2).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[2] = false; 
     }

}   

void LogicalCamera::logical_camera_station2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 2: '" << image_msg->models.size() << "' objects.");
    if (get_cam[3])
     { 
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
        
        std::string frame_name = "logical_camera_station2_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(3).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[3] = false; 
     }

}

void LogicalCamera::logical_camera_station3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 3: '" << image_msg->models.size() << "' objects.");
    if (get_cam[4])
     { 
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
        
        std::string frame_name = "logical_camera_station3_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(4).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[4] = false; 
     }

}

void LogicalCamera::logical_camera_station4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 4: '" << image_msg->models.size() << "' objects.");
    if (get_cam[5])
     { 
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
        
        std::string frame_name = "logical_camera_station4_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(5).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[5] = false; 
     }

}
void LogicalCamera::logical_camera_agv1as1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[6])
     { 
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
        
        std::string frame_name = "logical_camera_agv1as1_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(6).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[6] = false; 
     }
}

void LogicalCamera::logical_camera_agv1as2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[7])
     { 
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
        
        std::string frame_name = "logical_camera_agv1as2_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(7).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[7] = false; 
     }
}

void LogicalCamera::logical_camera_agv1ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[8])
     { 
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
        
        std::string frame_name = "logical_camera_agv1ks_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(8).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[8] = false; 
     }
}

void LogicalCamera::logical_camera_agv2as1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[9])
     { 
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
        
        std::string frame_name = "logical_camera_agv2as1_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(9).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[9] = false; 
     }
}

void LogicalCamera::logical_camera_agv2as2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[10])
     { 
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
        
        std::string frame_name = "logical_camera_agv2as2_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(10).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[10] = false; 
     }
}

void LogicalCamera::logical_camera_agv2ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[11])
     {
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
        
        std::string frame_name = "logical_camera_agv2ks_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(11).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[11] = false; 
     }
}

void LogicalCamera::logical_camera_agv3as3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[12])
     { 
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
        
        std::string frame_name = "logical_camera_agv3as3_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(12).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[12] = false; 
     }
}

void LogicalCamera::logical_camera_agv3as4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[13])
     { 
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
        
        std::string frame_name = "logical_camera_agv3as4_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(13).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[13] = false; 
     }
}

void LogicalCamera::logical_camera_agv3ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[14])
     { 
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
        
        std::string frame_name = "logical_camera_agv3ks_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(14).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[14] = false; 
     }
}

void LogicalCamera::logical_camera_agv4as3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[15])
     { 
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
        
        std::string frame_name = "logical_camera_agv4as3_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(15).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[15] = false; 
     }
}

void LogicalCamera::logical_camera_agv4as4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[16])
     { 
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
        
        std::string frame_name = "logical_camera_agv4as4_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(16).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[16] = false; 
     }
}

void LogicalCamera::logical_camera_agv4ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[17])
     { 
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
        
        std::string frame_name = "logical_camera_agv4ks" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(17).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[17] = false; 
     }
}

void LogicalCamera::logical_camera_belt_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[18])
     { 
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
        
        std::string frame_name = "logical_camera_belt_" + image_msg->models.at(i).type + "_" + std::to_string(part_index) + "_frame";
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

 
        camera_parts_list.at(18).push_back(product);

        i++;
        part_index++; 
      }
      
     get_cam[18] = false; 
     }
}