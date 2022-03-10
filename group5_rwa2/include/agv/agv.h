#ifndef AGV_H
#define AGV_H
#include "../util/util.h"


class Agv 
{
  public:
     explicit Agv(ros::NodeHandle &);

    /// Called when a new String message from /ariac/agv1/state is received.
    void agv1_state_callback(const std_msgs::String::ConstPtr & msg);
   
    /// Called when a new String message from /ariac/agv2/state is received.
    void agv2_state_callback(const std_msgs::String::ConstPtr & msg);

    /// Called when a new String message from /ariac/agv3/state is received.
    void agv3_state_callback(const std_msgs::String::ConstPtr & msg);

    /// Called when a new String message from /ariac/agv4/state is received.
    void agv4_state_callback(const std_msgs::String::ConstPtr & msg);

    /// Called when a new String message from /ariac/agv1/station is received.
    void agv1_station_callback(const std_msgs::String::ConstPtr & msg);

    /// Called when a new String message from /ariac/agv2/station is received.
    void agv2_station_callback(const std_msgs::String::ConstPtr & msg);

    /// Called when a new String message from /ariac/agv3/station is received.
    void agv3_station_callback(const std_msgs::String::ConstPtr & msg);

    /// Called when a new String message from /ariac/agv4/station is received.
    void agv4_station_callback(const std_msgs::String::ConstPtr & msg);

    /**
     * @brief Get the agv1 station object
     * 
     * @return std::string agv1_station
     */
    std::string get_agv1_station();

    /**
     * @brief Get the agv2 station object
     * 
     * @return std::string agv2_station
     */
    std::string get_agv2_station();

    /**
     * @brief Get the agv3 station object
     * 
     * @return std::string agv3_station
     */
    std::string get_agv3_station();

    /**
     * @brief Get the agv4 station object
     * 
     * @return std::string agv4_station
     */
    std::string get_agv4_station();

    /**
     * @brief Ships the shipment from agv to assembly station.
     * 
     */
    bool shipAGV(std::string, std::string, std::string);

 
  private:
    std::string station_;
    std::string shipment_id_;
    ros::Subscriber agv1_state_subscriber;
    ros::Subscriber agv2_state_subscriber;
    ros::Subscriber agv3_state_subscriber;
    ros::Subscriber agv4_state_subscriber;
    ros::Subscriber agv1_station_subscriber;
    ros::Subscriber agv2_station_subscriber;
    ros::Subscriber agv3_station_subscriber;
    ros::Subscriber agv4_station_subscriber;
    ros::ServiceClient agv1_client_;
    ros::ServiceClient agv2_client_;
    ros::ServiceClient agv3_client_;
    ros::ServiceClient agv4_client_;
    std::string agv1_station_;
    std::string agv2_station_;
    std::string agv3_station_;
    std::string agv4_station_;

};


#endif