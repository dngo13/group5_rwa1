#include "../include/comp/comp_class.h"


void MyCompetitionClass::current_score_callback(const std_msgs::Float32::ConstPtr & msg)
  {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

void MyCompetitionClass::competition_state_callback(const std_msgs::String::ConstPtr & msg)
  {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

void MyCompetitionClass::order_callback(const nist_gear::Order::ConstPtr & order_msg)
  {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    
    received_orders_.push_back(*order_msg);
    
    ROS_INFO_STREAM("" << received_orders_.at(0).order_id);
    
    // Creating instance of struct Order.
    Order new_order;
    new_order.order_id = order_msg->order_id;

    for (const auto &kit: order_msg->kitting_shipments){
        // Creating instance of struct Kitting.
        Kitting new_kitting;
        new_kitting.agv_id = kit.agv_id;
        new_kitting.shipment_type = kit.shipment_type;
        new_kitting.station_id = kit.station_id;
        for (const auto &Prod: kit.products){
            // Creating instance of struct Product.
            Product new_kproduct;
            new_kproduct.type = Prod.type;
            new_kproduct.frame_pose = Prod.pose;
            new_kitting.products.push_back(new_kproduct);
        }
        new_order.kitting.push_back(new_kitting);
    }

    for (const auto &asmb: order_msg->assembly_shipments){
        // Creating instance of struct Assembly.
        Assembly new_assembly;
        new_assembly.shipment_type = asmb.shipment_type;
        new_assembly.stations = asmb.station_id;
        for (const auto &Prod: asmb.products){
            // Creating instance of struct Product.
            Product new_aproduct;
            new_aproduct.type = Prod.type;
            new_aproduct.frame_pose = Prod.pose;
            new_assembly.products.push_back(new_aproduct);
        }
        new_order.assembly.push_back(new_assembly);
    }
   
    order_list_.push_back(new_order);
  }

std::string MyCompetitionClass::getCompetitionState(){
    return competition_state_;
}  

void MyCompetitionClass::process_order(){
      auto current_order = order_list_.front();
      auto current_kitting = current_order.kitting.front();
      auto kproduct_list = current_kitting.products;
      auto current_assembly = current_order.assembly.front();
      auto aproduct_list = current_assembly.products;

      for (const auto &kp: kproduct_list){
          kproduct_list.push_back(kp);
      }
      kproduct_list_ = kproduct_list;

      for (const auto &ap: aproduct_list){
          aproduct_list.push_back(ap);
      }
      aproduct_list_ = aproduct_list;
  }

std::vector<Order> MyCompetitionClass::get_order_list(){
      return order_list_;
  }

std::vector<Product> MyCompetitionClass::get_product_list(){
      return kproduct_list_, aproduct_list_;
  }

void MyCompetitionClass::logical_camera_station2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
{
  ROS_INFO_STREAM_THROTTLE(10,
    "Logical camera station 2: '" << image_msg->models.size() << "' objects.");
}

void MyCompetitionClass::logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
{
  ROS_INFO_STREAM_THROTTLE(10,"Logical camera bins8: '" << image_msg->models.size() << "' objects.");
}

void MyCompetitionClass::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
{
  if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected by Quality sensor 1");
  }
  
}

void MyCompetitionClass::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
{
  if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected by Quality sensor 2");
  }
  
}

void MyCompetitionClass::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
{
  if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected by Quality sensor 3");
  }

}

void MyCompetitionClass::quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
{
  if (!image_msg->models.empty()){
    ROS_INFO_STREAM_THROTTLE(10,"Faulty part detected by Quality sensor 4");
  }
}

void MyCompetitionClass::breakbeam0_callback(const nist_gear::Proximity::ConstPtr & msg) 
  {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    }
  }

void MyCompetitionClass::proximity_sensor0_callback(const sensor_msgs::Range::ConstPtr & msg)
{
  if ((msg->max_range - msg->range) > 0.01)
  {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

void MyCompetitionClass::laser_profiler0_callback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f)
    {
      return std::isfinite(f);
      });
  if (number_of_valid_ranges > 0)
  {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}

void MyCompetitionClass::agv1_station_callback(const std_msgs::String::ConstPtr & msg)
{
  msg->data;
}

void MyCompetitionClass::agv2_station_callback(const std_msgs::String::ConstPtr & msg)
{
  msg->data;
}

void MyCompetitionClass::agv3_station_callback(const std_msgs::String::ConstPtr & msg)
{
  msg->data;
}

void MyCompetitionClass::agv4_station_callback(const std_msgs::String::ConstPtr & msg)
{
  msg->data;
}

std::string MyCompetitionClass::get_agv_id()
{
  return order_list_.at(0).kitting.at(0).agv_id;
}

void MyCompetitionClass::callback60(const ros::TimerEvent& event){
  wait60 = true;
}

bool MyCompetitionClass::get_timer(){
  return wait60;
}