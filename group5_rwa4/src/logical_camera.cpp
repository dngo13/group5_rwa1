#include "../include/camera/logical_camera.h"

LogicalCamera::LogicalCamera(ros::NodeHandle & node) 
: tfBuffer(), tfListener(tfBuffer)
{
    node_ = node;

}

void LogicalCamera::callback(const ros::TimerEvent& event){
  wait = false;
}

bool LogicalCamera::get_timer(){
  return wait;
}


void LogicalCamera::logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
     blackout_time_ = ros::Time::now().toSec(); 
     if (get_cam[0])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_bins0";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        if(world_pose.position.x > -2.28 && world_pose.position.y > 2.96){
          product.bin_number = 1;
          bins_list.at(0).push_back(product);
        }
        else if( world_pose.position.x > -2.28 && world_pose.position.y < 2.96){
          product.bin_number = 2;
          bins_list.at(1).push_back(product);
        }
        else if(world_pose.position.x < -2.28 && world_pose.position.y < 2.96){
          product.bin_number = 3;
          bins_list.at(2).push_back(product);
        }
        else if(world_pose.position.x < -2.28 && world_pose.position.y > 2.96){
          product.bin_number = 4;
          bins_list.at(3).push_back(product);
        }
        camera_parts_list.at(0).push_back(product);
        i++; 
      }
     get_cam[0] = false; 
     }
}


void LogicalCamera::logical_camera_bins1_callback(
  const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    blackout_time_ = ros::Time::now().toSec();
    if (get_cam[1])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0}; 
      while(i < image_msg->models.size()){
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_bins1";
        product.status = "free"; 
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        if(world_pose.position.x > -2.28 && world_pose.position.y < -2.96){
          product.bin_number = 5;
          bins_list.at(4).push_back(product);
        }
        else if(world_pose.position.x > -2.28 && world_pose.position.y > -2.96){
          product.bin_number = 6;
          bins_list.at(5).push_back(product);
        }
        else if(world_pose.position.x < -2.28 && world_pose.position.y > -2.96){
          product.bin_number = 7;
          bins_list.at(6).push_back(product);
        }
        else if(world_pose.position.x < -2.28 && world_pose.position.y < -2.96){
          product.bin_number = 8;
          bins_list.at(7).push_back(product);
        }
        camera_parts_list.at(1).push_back(product);
        i++;
      }
     get_cam[1] = false; 
     }
}

std::array<std::vector<Product>,19> LogicalCamera::findparts(){
  ROS_INFO_STREAM("In Findparts");
  for (int i{0}; i < 18; i++){
    get_cam[i] = true;
    camera_parts_list.at(i).clear();
  }
  get_cam[18] = false;

  ros::Subscriber logical_camera_bins0_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins0", 2, 
    &LogicalCamera::logical_camera_bins0_callback, this);

  ros::Subscriber logical_camera_bins1_subscriber = node_.subscribe(
    "/ariac/logical_camera_bins1", 2, 
    &LogicalCamera::logical_camera_bins1_callback, this);

  // ros::Subscriber logical_camera_station1_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_station1", 2, 
  //   &LogicalCamera::logical_camera_station1_callback, this);

  // ros::Subscriber logical_camera_station2_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_station2", 2, 
  //   &LogicalCamera::logical_camera_station2_callback, this);

  // ros::Subscriber logical_camera_station3_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_station3", 2, 
  //   &LogicalCamera::logical_camera_station3_callback, this);

  // ros::Subscriber logical_camera_station4_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_station4", 2, 
  //   &LogicalCamera::logical_camera_station4_callback, this);

  ros::Subscriber logical_camera_agv1as1_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv1as1", 2, 
    &LogicalCamera::logical_camera_agv1as1_callback, this);

  ros::Subscriber logical_camera_agv1as2_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv1as2", 2, 
    &LogicalCamera::logical_camera_agv1as2_callback, this);

  // ros::Subscriber logical_camera_agv1ks_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_agv1ks", 2, 
  //   &LogicalCamera::logical_camera_agv1ks_callback, this);

  ros::Subscriber logical_camera_agv2as1_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv2as1", 2, 
    &LogicalCamera::logical_camera_agv2as1_callback, this);

  ros::Subscriber logical_camera_agv2as2_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv2as2", 2, 
    &LogicalCamera::logical_camera_agv2as2_callback, this);

  // ros::Subscriber logical_camera_agv2ks_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_agv2ks", 2, 
  //   &LogicalCamera::logical_camera_agv2ks_callback, this);

  ros::Subscriber logical_camera_agv3as3_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv3as3", 2, 
    &LogicalCamera::logical_camera_agv3as3_callback, this);

  ros::Subscriber logical_camera_agv3as4_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv3as4", 2, 
    &LogicalCamera::logical_camera_agv3as4_callback, this);

  // ros::Subscriber logical_camera_agv3ks_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_agv3ks", 2, 
  //   &LogicalCamera::logical_camera_agv3ks_callback, this);

  ros::Subscriber logical_camera_agv4as3_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv4as3", 2, 
    &LogicalCamera::logical_camera_agv4as3_callback, this);

  ros::Subscriber logical_camera_agv4as4_subscriber = node_.subscribe(
    "/ariac/logical_camera_agv4as4", 2, 
    &LogicalCamera::logical_camera_agv4as4_callback, this);

  // ros::Subscriber logical_camera_agv4ks_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_agv4ks", 2, 
  //   &LogicalCamera::logical_camera_agv4ks_callback, this);

  // ros::Subscriber logical_camera_belt_subscriber = node_.subscribe(
  //   "/ariac/logical_camera_belt", 2, 
  //   &LogicalCamera::logical_camera_belt_callback, this);

  ros::Duration(sleep(5.0));
  return camera_parts_list;  

}

void LogicalCamera::segregate_parts(std::array<std::vector<Product>,19> list){
  camera_map_.clear();
  for (auto &l: list){
    for(auto &part: l){
      camera_map_[part.type].push_back(part);
    }
  }
}

std::vector<int> LogicalCamera::get_ebin_list(){
  for (int i = 0; i < 8; i++){
    if(bins_list.at(i).size() == 0){
      empty_bin.push_back(i+1);
    }
  }
  return empty_bin;
}

double LogicalCamera::CheckBlackout(){
  return blackout_time_;
}


void LogicalCamera::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[0]){
      if (!image_msg->models.empty()){
        ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv1");

        for (auto &model:image_msg->models){
          Product product;
          product.type = model.type;
          product.frame_pose = model.pose;
          product.camera = "quality_control_sensor_1";
          auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
          product.world_pose = world_pose;
          product.faulty_cam_agv = "agv1";
          faulty_part_list_.push_back(product);
        }
      }
      get_faulty_cam[0] = false; 
    }
}


void LogicalCamera::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[1]){
      if (!image_msg->models.empty()){
        ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv2");
    
        for (auto &model:image_msg->models){
          Product product;
          product.type = model.type;
          product.frame_pose = model.pose;
          product.camera = "quality_control_sensor_2";
          auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
          product.world_pose = world_pose;
          product.faulty_cam_agv = "agv2";
          faulty_part_list_.push_back(product);
        }
      }

    get_faulty_cam[1] = false;
  }
}


void LogicalCamera::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[2]){
      if (!image_msg->models.empty()){
        ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv3");

        for (auto &model:image_msg->models){
          Product product;
          product.type = model.type;
          product.frame_pose = model.pose;
          product.camera = "quality_control_sensor_3";
          auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
          product.world_pose = world_pose;
          product.faulty_cam_agv = "agv3";
          faulty_part_list_.push_back(product);
        }
      }

      get_faulty_cam[2] = false;
    }
}


void LogicalCamera::quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (get_faulty_cam[3]){
      if (!image_msg->models.empty()){
        ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected on agv4");

        for (auto &model:image_msg->models){
          Product product;
          product.type = model.type;
          product.frame_pose = model.pose;
          product.camera = "quality_control_sensor_4";
          auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
          product.world_pose = world_pose;
          product.faulty_cam_agv = "agv4";
          faulty_part_list_.push_back(product);
        }
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

void LogicalCamera::query_faulty_cam(){
  faulty_part_list_.clear();
  for (int j{0}; j <= 3; j++){  
    get_faulty_cam[j] = true;
  }
  
}

void LogicalCamera::logical_camera_station1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    // ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 1: '" << image_msg->models.size() << "' objects.");
    if (get_cam[2])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_station1";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(2).push_back(product);
        i++;
      }
     get_cam[2] = false; 
     }
}   

void LogicalCamera::logical_camera_station2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    // ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 2: '" << image_msg->models.size() << "' objects.");
    if (get_cam[3])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_station2";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(3).push_back(product);
        i++;
      }
     get_cam[3] = false; 
     }
}

void LogicalCamera::logical_camera_station3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    // ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 3: '" << image_msg->models.size() << "' objects.");
    if (get_cam[4])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_station3";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(4).push_back(product);
        i++;
      }
     get_cam[4] = false; 
     }

}

void LogicalCamera::logical_camera_station4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    // ROS_INFO_STREAM_THROTTLE(10,"Logical camera station 4: '" << image_msg->models.size() << "' objects.");
    if (get_cam[5])
     { 
      ros::Duration timeout(5.0);

      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_station4";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(5).push_back(product);
        i++;
      }
      
     get_cam[5] = false; 
     }

}
void LogicalCamera::logical_camera_agv1as1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[6])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0}; 
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv1as1";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(6).push_back(product);
        i++;
      }
      
     get_cam[6] = false; 
     }
}

void LogicalCamera::logical_camera_agv1as2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[7])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv1as2";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(7).push_back(product);
        i++;
      }
      
     get_cam[7] = false; 
     }
}

void LogicalCamera::logical_camera_agv1ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[8])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv1ks";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(8).push_back(product);
        i++;
      }     
     get_cam[8] = false; 
     }
}

void LogicalCamera::logical_camera_agv2as1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[9])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv2as1";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(9).push_back(product);
        i++;
      }
      
     get_cam[9] = false; 
     }
}

void LogicalCamera::logical_camera_agv2as2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[10])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv2as2";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(10).push_back(product);
        i++;
      }
     get_cam[10] = false; 
     }
}

void LogicalCamera::logical_camera_agv2ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[11])
     {
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        std::string product_type = image_msg->models.at(i).type;
        Product product;
        product.type = product_type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv2ks";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(11).push_back(product);
        i++; 
      }
     get_cam[11] = false; 
     }
}

void LogicalCamera::logical_camera_agv3as3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[12])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv3as3";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(12).push_back(product);
        i++;
      }
     get_cam[12] = false; 
     }
}

void LogicalCamera::logical_camera_agv3as4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[13])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0}; 
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv3as4";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(13).push_back(product);
        i++;
      }
     get_cam[13] = false; 
     }
}

void LogicalCamera::logical_camera_agv3ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[14])
     { 
      ros::Duration timeout(5.0);

      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv3ks";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(14).push_back(product);
        i++;
      }
     get_cam[14] = false; 
     }
}

void LogicalCamera::logical_camera_agv4as3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[15])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0}; 
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv4as3";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(15).push_back(product);
        i++; 
      }
     get_cam[15] = false; 
     }
}

void LogicalCamera::logical_camera_agv4as4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[16])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv4as4";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(16).push_back(product);
        i++;
      }
     get_cam[16] = false; 
     }
}

void LogicalCamera::logical_camera_agv4ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[17])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_agv4ks";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(17).push_back(product);
        i++;
      }
     get_cam[17] = false; 
     }
}

void LogicalCamera::logical_camera_belt_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  if (get_cam[18])
     { 
      ros::Duration timeout(5.0);
      unsigned short int i{0};
      while(i < image_msg->models.size())
      {
        Product product;
        product.type = image_msg->models.at(i).type;
        product.frame_pose = image_msg->models.at(i).pose;
        product.camera = "logical_camera_belt";
        product.status = "free";
        auto world_pose = motioncontrol::gettransforminWorldFrame(product.frame_pose, product.camera);
        product.world_pose = world_pose;
        camera_parts_list.at(18).push_back(product);
        i++;
      }
     get_cam[18] = false; 
     }
}