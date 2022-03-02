#include "../include/agv/agv.h"
 
 
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

void::Agv::submit_shipment(std::string agv_id, std::string st, std::string asn){
        nist_gear::AGVToAssemblyStation srv;
        srv.request.shipment_type = st;
        srv.request.assembly_station_name = asn;

        if (agv_id == "agv1"){
            client1.call(srv);
        }
        if (agv_id == "agv2"){
            client2.call(srv);
        }
        if (agv_id == "agv3"){
            client3.call(srv);
        }
        if (agv_id == "agv4"){
            client4.call(srv);
        }



        
}
