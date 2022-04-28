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
    
    // Subscribe to the '/ariac/logical_camera_agv1as1' topic.
    void logical_camera_agv1as1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv1as2' topic.
    void logical_camera_agv1as2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv1ks' topic.
    void logical_camera_agv1ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv2as1' topic.
    void logical_camera_agv2as1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv2as2' topic.
    void logical_camera_agv2as2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv2ks' topic.
    void logical_camera_agv2ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv3as3' topic.
    void logical_camera_agv3as3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv3as4' topic.
    void logical_camera_agv3as4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv3ks' topic.
    void logical_camera_agv3ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv4as3' topic.
    void logical_camera_agv4as3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv4as4' topic.
    void logical_camera_agv4as4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_agv4ks' topic.
    void logical_camera_agv4ks_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/logical_camera_belt' topic.
    void logical_camera_belt_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/quality_control_sensor_1' topic.
    void quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);

    // Subscribe to the '/ariac/quality_control_sensor_2' topic.
    void quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/quality_control_sensor_3' topic.
    void quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
    
    // Subscribe to the '/ariac/quality_control_sensor_4' topic.
    void quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg);
 
    // List of all the models found by the logical cameras.
    std::array<std::vector<Product>,19> camera_parts_list;

    // Buffer for transform.
    tf2_ros::Buffer tfBuffer;

    // Tf Listener
    tf2_ros::TransformListener tfListener;

    // Array of boolean to check the camera data only once when needed. 
    bool get_cam[19] = {true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true};

    // Array of boolean to check the quality control sensor data only once when needed.
    bool get_faulty_cam[4] = {true,true,true,true};

    /// callback for timer
    void callback(const ros::TimerEvent& event);

    /// Accessor for boolean check of timer
    bool get_timer();

    std::vector<Product> faulty_part_list_;
    
    /**
     * @brief Detects the parts in vicinity of all the logical cameras and stores data of each model. 
     * 
     * @return std::array<std::vector<Product>,19> 
     */
    std::array<std::vector<Product>,19> findparts();
    /**
     * @brief Populates the map according to product type
     * 
     * @param list List of parts seen by by logical cameras 
     */
    void segregate_parts(std::array<std::vector<Product>,19> list);
    /**
     * @brief Get the list of faulty parts
     * 
     * @return std::vector<Product> List of faulty parts
     */
    std::vector<Product> get_faulty_part_list();  
    /**
     * @brief Checks if there is a sensor blackout.
     * 
     * @return true 
     * @return false 
     */
    double CheckBlackout();
    /**
     * @brief Query the quality control sensors to check for faulty parts, if any
     * 
     */
    void query_faulty_cam();
    /**
     * @brief Get the generated map of parts 
     * 
     * @return std::map<std::string, std::vector<Product> >&  Map of parts
     */
    std::map<std::string, std::vector<Product> >& get_camera_map(){
        return camera_map_;
    }

    std::array<std::vector<Product>,8> get_bin_list();

    std::vector<int> get_ebin_list();

    private:
    ros::NodeHandle node_;
    ros::Subscriber quality_control_sensor1_subscriber;
    ros::Subscriber quality_control_sensor2_subscriber;
    ros::Subscriber quality_control_sensor3_subscriber;
    ros::Subscriber quality_control_sensor4_subscriber;
    bool logflag_{};
    ros::Timer timer;
    bool wait{false};
    std::map<std::string, std::vector<Product> > camera_map_;
    double blackout_time_ = 0;
    std::array<std::vector<Product>,8> bins_list;
    std::vector<int> empty_bin;
    
};


#endif