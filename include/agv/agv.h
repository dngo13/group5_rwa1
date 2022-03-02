#ifndef AGV_H
#define AGV_H
#include "../util/util.h"


class Agv 
{
  public:
     explicit Agv(ros::NodeHandle & node){
        client1 = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv1/submit_shipment");
        client2 = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv2/submit_shipment");
        client3 = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv3/submit_shipment");
        client4 = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv4/submit_shipment");
        

        // Subscribe to the '/ariac/agv1/state' topic.
        agv1_state_subscriber = node.subscribe("/ariac/agv1/state", 10, &Agv::agv1_state_callback, this);

        // Subscribe to the '/ariac/agv2/state' topic.
        agv2_state_subscriber = node.subscribe("/ariac/agv2/state", 10,&Agv::agv2_state_callback, this);

        // Subscribe to the '/ariac/agv3/state' topic.
        agv3_state_subscriber = node.subscribe("/ariac/agv3/state", 10, &Agv::agv3_state_callback, this);

        // Subscribe to the '/ariac/agv4/state' topic.
        agv4_state_subscriber = node.subscribe("/ariac/agv4/state", 10, &Agv::agv4_state_callback, this);   

        // Subscribe to the '/ariac/agv1/station' topic.
        agv1_station_subscriber = node.subscribe("ariac/agv1/station", 1, &Agv::agv1_station_callback, this);

        // Subscribe to the '/ariac/agv2/station' topic.
        agv2_station_subscriber = node.subscribe("ariac/agv2/station", 1, &Agv::agv2_station_callback, this);

        // Subscribe to the '/ariac/agv3/station' topic.
        agv3_station_subscriber = node.subscribe("ariac/agv3/station", 1, &Agv::agv3_station_callback, this);

        // Subscribe to the '/ariac/agv4/station' topic.
        agv4_station_subscriber = node.subscribe("ariac/agv4/station", 1, &Agv::agv4_station_callback, this);

    }

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
    void submit_shipment(std::string, std::string, std::string);

 
  private:
    std::string station;
    std::string shipment_id;
    ros::Subscriber agv1_state_subscriber;
    ros::Subscriber agv2_state_subscriber;
    ros::Subscriber agv3_state_subscriber;
    ros::Subscriber agv4_state_subscriber;
    ros::Subscriber agv1_station_subscriber;
    ros::Subscriber agv2_station_subscriber;
    ros::Subscriber agv3_station_subscriber;
    ros::Subscriber agv4_station_subscriber;
    ros::ServiceClient client1;
    ros::ServiceClient client2;
    ros::ServiceClient client3;
    ros::ServiceClient client4;
    std::string agv1_station_;
    std::string agv2_station_;
    std::string agv3_station_;
    std::string agv4_station_;

};


#endif