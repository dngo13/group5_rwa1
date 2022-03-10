#include "../include/agv/agv.h"
 
 
Agv::Agv(ros::NodeHandle & node){
    agv1_client_ = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv1/submit_shipment");
    agv2_client_ = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv2/submit_shipment");
    agv3_client_ = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv3/submit_shipment");
    agv4_client_ = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv4/submit_shipment");
    
    if (!agv1_client_.exists()) {
        ROS_INFO("Waiting for the AGV1 to be ready...");
        agv1_client_.waitForExistence();
        ROS_INFO("AGV1 is now ready.");
    }

    if (!agv2_client_.exists()) {
        ROS_INFO("Waiting for the AGV2 to be ready...");
        agv1_client_.waitForExistence();
        ROS_INFO("AGV2 is now ready.");
    }

    if (!agv3_client_.exists()) {
        ROS_INFO("Waiting for the AGV3 to be ready...");
        agv1_client_.waitForExistence();
        ROS_INFO("AGV3 is now ready.");
    }

    if (!agv4_client_.exists()) {
        ROS_INFO("Waiting for the AGV4 to be ready...");
        agv1_client_.waitForExistence();
        ROS_INFO("AGV4 is now ready.");
    }

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


void Agv::agv1_state_callback(const std_msgs::String::ConstPtr & msg)
{
     msg->data;
}

void Agv::agv2_state_callback(const std_msgs::String::ConstPtr & msg)
{
    msg->data;
}

void Agv::agv3_state_callback(const std_msgs::String::ConstPtr & msg)
{
    msg->data;
}

void Agv::agv4_state_callback(const std_msgs::String::ConstPtr & msg)
{
    msg->data;
}

void Agv::agv1_station_callback(const std_msgs::String::ConstPtr & msg)
{
    agv1_station_ = msg->data;
}

void Agv::agv2_station_callback(const std_msgs::String::ConstPtr & msg)
{
    agv2_station_ = msg->data;
}

void Agv::agv3_station_callback(const std_msgs::String::ConstPtr & msg)
{
    agv3_station_ = msg->data;
}

void Agv::agv4_station_callback(const std_msgs::String::ConstPtr & msg)
{
    agv4_station_ = msg->data;
}

std::string Agv::get_agv1_station(){
    return agv1_station_;
}

std::string Agv::get_agv2_station(){
    return agv2_station_;
}

std::string Agv::get_agv3_station(){
    return agv3_station_;
}

std::string Agv::get_agv4_station(){
    return agv4_station_;
}

bool Agv::shipAGV(std::string agv_id, std::string st, std::string asn){
        nist_gear::AGVToAssemblyStation srv;
        srv.request.shipment_type = st;
        srv.request.assembly_station_name = asn;

        if (agv_id == "agv1"){
            agv1_client_.call(srv);
        }
        if (agv_id == "agv2"){
            agv2_client_.call(srv);
        }
        if (agv_id == "agv3"){
            agv3_client_.call(srv);
        }
        if (agv_id == "agv4"){
            agv4_client_.call(srv);
        }

        if (srv.response.success) {
        ROS_INFO_STREAM("[agv_control][sendAGV] " + agv_id + "is taking order: " + srv.request.shipment_type);
        return true;
        }
        else {
        ROS_ERROR_STREAM("[agv_control][sendAGV] Failed to call " + agv_id);
        return false;
        }
}

