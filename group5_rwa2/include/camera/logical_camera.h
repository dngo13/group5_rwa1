#ifndef LOGICAL_CAMERA_H
#define LOGICAL_CAMERA_H
#include "../util/util.h"

class LogicalCamera
{
    public:
    explicit LogicalCamera(ros::NodeHandle &);

    /// Called when a new LogicalCameraImage message from /ariac/logical_camera_bins0 is received.
    void logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    /// Called when a new LogicalCameraImage message from /ariac/logical_camera_bins1 is received.
    void logical_camera_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);


    // Subscribe to the '/ariac/logical_camera_station1' topic.
    void logical_camera_station1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/logical_camera_station2' topic.
    void logical_camera_station2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/logical_camera_station3' topic.
    void logical_camera_station3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/logical_camera_station4' topic.
    void logical_camera_station4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/quality_control_sensor_1' topic.
    void quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/quality_control_sensor_2' topic.
    void quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/quality_control_sensor_3' topic.
    void quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/quality_control_sensor_4' topic.
    void quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
 
    // void callback_b1(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // void callback_b2(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // std::string CheckBlackout();

    std::array<std::vector<Product>,2> findparts();

    std::array<std::vector<Product>,2> camera_parts_list;

    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener;

    bool get_bins0{true};

    bool get_bins1{true};

    std::vector<Product> product_list0_;
  
    std::vector<Product> product_list1_;

    std::vector<Product> get_product_list0();

    std::vector<Product> get_product_list1();

    // std::string flag1{};

    // std::string flag2{};


    private:
    ros::NodeHandle node_;
    // ros::Subscriber logical_camera_bins0_subscriber;
    // ros::Subscriber logical_camera_bins1_subscriber;
    ros::Subscriber logical_camera_station1_subscriber;
    ros::Subscriber logical_camera_station2_subscriber;
    ros::Subscriber logical_camera_station3_subscriber;
    ros::Subscriber logical_camera_station4_subscriber;
    ros::Subscriber quality_control_sensor1_subscriber;
    ros::Subscriber quality_control_sensor2_subscriber;
    ros::Subscriber quality_control_sensor3_subscriber;
    ros::Subscriber quality_control_sensor4_subscriber;
    ros::Subscriber quality_control_sensor1_subscriber_b1;
    ros::Subscriber quality_control_sensor2_subscriber_b2;




};


#endif