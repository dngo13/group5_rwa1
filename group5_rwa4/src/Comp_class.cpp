#include "../include/comp/comp_class.h"

MyCompetitionClass::MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0)
  {
    node_ = node;
    gantry_arm_joint_trajectory_publisher_ = node_.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm1/arm/command", 10);

    kitting_arm_joint_trajectory_publisher_ = node_.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm2/arm/command", 10);
  }

void MyCompetitionClass::init() {
    double time_called = ros::Time::now().toSec();
    competition_start_time_ = ros::Time::now().toSec();

    // subscribe to the '/ariac/competition_state' topic.
    competition_state_subscriber_ = node_.subscribe(
        "/ariac/competition_state", 10, &MyCompetitionClass::competition_state_callback, this);

    // subscribe to the '/clock' topic.
    competition_clock_subscriber_ = node_.subscribe(
        "/clock", 10, &MyCompetitionClass::competition_clock_callback, this);
    
    // Subscribe to the '/ariac/current_score' topic.
    current_score_subscriber_ = node_.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, this);

    // Subscribe to the '/ariac/orders' topic.
    orders_subscriber = node_.subscribe(
    "/ariac/orders", 1,
    &MyCompetitionClass::order_callback, this);
    
    logical_camera_subscriber_ = node_.subscribe(
    "/ariac/logical_camera_bins0", 1, 
    &MyCompetitionClass::logical_camera_callback, this);
    
    break_beam_subscriber_ = node_.subscribe(
    "/ariac/breakbeam_0_change", 1, 
    &MyCompetitionClass::breakbeam0_callback, this);

    // Timer at start
    timer = node_.createTimer(ros::Duration(2), &MyCompetitionClass::callback, this);
    
    // start the competition
    startCompetition();
}

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

////////////////////////
void MyCompetitionClass::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg) {
    competition_clock_ = msg->clock;
}


void MyCompetitionClass::startCompetition()
{
  // create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
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

void MyCompetitionClass::endCompetition()
{
  ros::ServiceClient end_client = node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("Competition ended!");
    ros::shutdown();
  }
}

////////////////////////
double MyCompetitionClass::getStartTime() {
    return competition_start_time_;
}

////////////////////////
double MyCompetitionClass::getClock() {
    double time_spent = competition_clock_.toSec();
    return time_spent;
}

////////////////////////
std::string MyCompetitionClass::getCompetitionState() {
    return competition_state_;
}

void MyCompetitionClass::order_callback(const nist_gear::Order::ConstPtr & order_msg)
  {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    
    // Creating instance of struct Order.
    Order new_order;
    new_order.order_id = order_msg->order_id;
    new_order.order_processed = false;
    new_order.priority = 1;
    
    if(new_order.order_id == "order_1"){
      // order1_announced = true;
      new_order.priority = 3;
      ROS_INFO("High priority order is announced ");
      high_priority_announced = true;
    }
  
    for (const auto &kit: order_msg->kitting_shipments){
        // Creating instance of struct Kitting.
        Kitting new_kitting;
        new_kitting.agv_id = kit.agv_id;
        new_kitting.shipment_type = kit.shipment_type;
        new_kitting.station_id = kit.station_id;
        new_kitting.kitting_done = false;

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
        new_assembly.asssembly_done = false;

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

std::vector<Order> MyCompetitionClass::get_order_list(){
      return order_list_;
  }


void MyCompetitionClass::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
  blackout_time_ = ros::Time::now().toSec();
}


double MyCompetitionClass::CheckBlackout(){
  return blackout_time_;
}

void MyCompetitionClass::breakbeam0_callback(const nist_gear::Proximity::ConstPtr & msg) 
  {
    blackout_time_ = ros::Time::now().toSec();
    if (msg->object_detected) {  
      parts_rolling_on_conveyor = true;
    }
  }

bool MyCompetitionClass::conveyor_check(){
  return parts_rolling_on_conveyor;
}

void MyCompetitionClass::proximity_sensor0_callback(const sensor_msgs::Range::ConstPtr & msg)
{
  if ((msg->max_range - msg->range) > 0.01){
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


void MyCompetitionClass::callback(const ros::TimerEvent& event){
  wait = true;
}


