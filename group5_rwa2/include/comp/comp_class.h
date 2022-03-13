#ifndef COMP_CLASS_H
#define COMP_CLASS_H
#include "../util/util.h"

class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node);

  /**
   * @brief Initialize the competition.
   * 
   */
  void init();

  /**
 * @brief Start the competition by waiting for and then calling the start ROS Service.
 * 
 */
  void startCompetition();

  /**
   * @brief End the competition by calling the service.
   * 
   */
  void endCompetition();

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg);

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg);
  /**
     * @brief Time since the competition started
     *
     * @param msg
     */
  void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg);
  /**
   * @brief Get the Clock objectGet
   *
   * @return double
   */
  double getClock();
  /**
   * @brief Get the Start Time object
   *
   * @return double
   */
  double getStartTime();

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
 
  // Called when a new LogicalCameraImage message from /ariac/depth_camera_bins1 is received.
  void depth_camera_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
  
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
  
  /// callback for timer
  void callback(const ros::TimerEvent& event);

  /// Accessor for boolean check of timer
  bool get_timer();

  /**
   * @brief Gets the agv id object
   * 
   * @return std::string agv_id
   */
  std::string get_agv_id();

  /**
   * @brief Get list of prothe product list0 object
   * 
   * @return std::vector<Product> 
   */


private:
  ros::NodeHandle node_;
  std::string competition_state_;
  double current_score_;
  ros::Time competition_clock_;
  double competition_start_time_;
  ros::Publisher gantry_arm_joint_trajectory_publisher_;
  ros::Publisher kitting_arm_joint_trajectory_publisher_;
  std::vector<nist_gear::Order> received_orders_;
  sensor_msgs::JointState gantry_arm_current_joint_states_;
  sensor_msgs::JointState kitting_arm_current_joint_states_;
  ros::Subscriber current_score_subscriber_;
  ros::Subscriber competition_state_subscriber_;
  ros::Subscriber competition_clock_subscriber_;
  ros::Subscriber orders_subscriber;
  std::vector<Order> order_list_;
  bool order_processed_;
  bool wait{false};
  
};

#endif