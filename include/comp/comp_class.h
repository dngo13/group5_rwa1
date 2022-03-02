#ifndef COMP_CLASS_H
#define COMP_CLASS_H
#include "../util/util.h"

class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0)
  {
    gantry_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm1/arm/command", 10);

    kitting_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm2/arm/command", 10);
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg);

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg);

  /// Accessor for competition state.
  std::string getCompetitionState();

  /// Called when a new Order message is received.
  void order_callback(const nist_gear::Order::ConstPtr & order_msg);

  /**
   * @brief Stores the kitting and assembly products in seperate lists. 
   * 
   */
  void process_order();

  /**
   * @brief Gets the order list object
   * 
   * @return std::vector<Order> 
   */
  std::vector<Order> get_order_list();

  /**
   * @brief Gets the product list object
   * 
   * @return std::vector<Product> 
   */
  std::vector<Product> get_product_list();
 
  /// Called when a new LogicalCameraImage message from /ariac/logical_camera_station2 is received.
  void logical_camera_station2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

  /// Called when a new LogicalCameraImage message from /ariac/logical_camera_bins0 is received.
  void logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

  /// Called when a new LogicalCameraImage message from /ariac/quality_control_sensor1 is received.
  void quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

  /// Called when a new LogicalCameraImage message from /ariac/quality_control_sensor2 is received.
  void quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

  /// Called when a new LogicalCameraImage message from /ariac/quality_control_sensor3 is received.
  void quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
  
  /// Called when a new LogicalCameraImage message from /ariac/quality_control_sensor4 is received.
  void quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
  
  /// Called when a new Proximity message from /ariac/breakbeam0 is received.
  void breakbeam0_callback(const nist_gear::Proximity::ConstPtr & msg);

  /// Called when a new Proximity message from /ariac/proximity_sensor_0 is received.
  void proximity_sensor0_callback(const sensor_msgs::Range::ConstPtr & msg);

  /// Called when a new LaserScan message from laser_profiler_0 is received.
  void laser_profiler0_callback(const sensor_msgs::LaserScan::ConstPtr & msg);

  /// Called when a new PointCloud message from depth_camera_bins1 is received.
  void depth_camera_bins1_callback(const sensor_msgs::PointCloud::ConstPtr & pc_msg);

  /// Called when a new String message from /ariac/agv1/station is received.
  void agv1_station_callback(const std_msgs::String::ConstPtr & msg);

  /// Called when a new String message from /ariac/agv2/station is received.
  void agv2_station_callback(const std_msgs::String::ConstPtr & msg);

  /// Called when a new String message from /ariac/agv3/station is received.
  void agv3_station_callback(const std_msgs::String::ConstPtr & msg);

  /// Called when a new String message from /ariac/agv4/station is received.
  void agv4_station_callback(const std_msgs::String::ConstPtr & msg);
  
  /**
   * @brief Gets the agv id object
   * 
   * @return std::string agv_id
   */
  std::string get_agv_id();

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher gantry_arm_joint_trajectory_publisher_;
  ros::Publisher kitting_arm_joint_trajectory_publisher_;
  std::vector<nist_gear::Order> received_orders_;
  sensor_msgs::JointState gantry_arm_current_joint_states_;
  sensor_msgs::JointState kitting_arm_current_joint_states_;
  std::vector<Order> order_list_;
  std::vector<Product> kproduct_list_;
  std::vector<Product> aproduct_list_;
};


#endif