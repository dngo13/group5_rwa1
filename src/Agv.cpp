#include "/home/darshan/ariac_ws/src/group5_rwa1/include/agv/agv.h"
 
 
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

std::string Agv::get_agv_id()
{
    return agv_id_;
}

void::Agv::submit_shipment(std::string agv_id_){
        nist_gear::AGVToAssemblyStation srv;

        if (agv_id_ == "agv1"){
            client1.call(srv);
        }
        if (agv_id_ == "agv2"){
            client2.call(srv);
        }
        if (agv_id_ == "agv3"){
            client3.call(srv);
        }
        if (agv_id_ == "agv4"){
            client4.call(srv);
        }

        
    }
