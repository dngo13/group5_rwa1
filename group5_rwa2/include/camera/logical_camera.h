#ifndef LOGICAL_CAMERA_H
#define LOGICAL_CAMERA_H
#include "../util/util.h"

class LogicalCamera
{
    public:
    explicit LogicalCamera(ros::NodeHandle &);

    // Subscribe to the '/ariac/logical_camera_bins0' topic.
    void logical_camera_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_bins1' topic.
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
    
    std::vector<Product> get_product_list0();

    std::vector<Product> get_product_list1();

    private:
    ros::Subscriber logical_camera_bins0_subscriber;
    ros::Subscriber logical_camera_bins1_subscriber;
    ros::Subscriber logical_camera_station1_subscriber;
    ros::Subscriber logical_camera_station2_subscriber;
    ros::Subscriber logical_camera_station3_subscriber;
    ros::Subscriber logical_camera_station4_subscriber;
    ros::Subscriber quality_control_sensor1_subscriber;
    ros::Subscriber quality_control_sensor2_subscriber;
    ros::Subscriber quality_control_sensor3_subscriber;
    ros::Subscriber quality_control_sensor4_subscriber;
    std::vector<Product> product_list0_;
    std::vector<Product> product_list1_;





};


#endif