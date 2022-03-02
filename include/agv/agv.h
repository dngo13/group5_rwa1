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

        agv1_station_subscriber = node.subscribe("ariac/agv1/station", 10, &Agv::agv1_station_callback, this);

        agv2_station_subscriber = node.subscribe("ariac/agv2/station", 10, &Agv::agv2_station_callback, this);

        agv3_station_subscriber = node.subscribe("ariac/agv3/station", 10, &Agv::agv3_station_callback, this);

        agv4_station_subscriber = node.subscribe("ariac/agv4/station", 10, &Agv::agv4_station_callback, this);

    }

    /// Called when a new LogicalCameraImage message is received.
    void agv1_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv2_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv3_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv4_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv1_station_callback(const std_msgs::String::ConstPtr & msg);

    void agv2_station_callback(const std_msgs::String::ConstPtr & msg);

    void agv3_station_callback(const std_msgs::String::ConstPtr & msg);

    void agv4_station_callback(const std_msgs::String::ConstPtr & msg);

    std::string get_agv1_station();

    std::string get_agv2_station();

    std::string get_agv3_station();

    std::string get_agv4_station();


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