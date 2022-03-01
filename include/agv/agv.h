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
    }

    /// Called when a new LogicalCameraImage message is received.
    void agv1_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv2_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv3_state_callback(const std_msgs::String::ConstPtr & msg);

    void agv4_state_callback(const std_msgs::String::ConstPtr & msg);

    void submit_shipment(std::string agv_id_);

    std::string get_agv_id();
 
  private:
    std::string station;
    std::string shipment_id;
    ros::Subscriber agv1_state_subscriber;
    ros::Subscriber agv2_state_subscriber;
    ros::Subscriber agv3_state_subscriber;
    ros::Subscriber agv4_state_subscriber;
    ros::ServiceClient client1;
    ros::ServiceClient client2;
    ros::ServiceClient client3;
    ros::ServiceClient client4;
    std::string agv_id_;
};


#endif