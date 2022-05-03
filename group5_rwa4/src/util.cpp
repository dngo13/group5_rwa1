#include "../include/util/util.h"
#include <stdlib.h>

namespace motioncontrol {

    void print(const tf2::Quaternion& quat) {
        ROS_INFO("[x: %f, y: %f, z: %f, w: %f]",
            quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    void print(const geometry_msgs::Pose& pose) {
        auto rpy = eulerFromQuaternion(pose);

        ROS_INFO("position: [x: %f, y: %f, z: %f]\norientation(quat): [x: %f, y: %f, z: %f, w: %f]\norientation(rpy): [roll: %f, pitch: %f, yaw: %f]",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            rpy[0], rpy[1], rpy[2]);
    }

    tf2::Quaternion quaternionFromEuler(double r, double p, double y) {
        tf2::Quaternion q;
        q.setRPY(r, p, y);
        // ROS_INFO("quat: [%f, %f, %f, %f]",
        //     q.getX(),
        //     q.getY(),
        //     q.getZ(),
        //     q.getW());

        return q;
    }

    std::array<double, 3> eulerFromQuaternion(const tf2::Quaternion& quat) {

        tf2::Quaternion q(
            quat.getX(),
            quat.getY(),
            quat.getZ(),
            quat.getW());
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("[%f, %f, %f]", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    std::array<double, 3> eulerFromQuaternion(
        const geometry_msgs::Pose& pose) {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("[%f, %f, %f]", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    std::array<double, 3> eulerFromQuaternion(
        double x, double y, double z, double w) {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("%f, %f, %f", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    geometry_msgs::Pose transformToWorldFrame(std::string part_in_camera_frame) {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);


        geometry_msgs::TransformStamped world_target_tf;
        geometry_msgs::TransformStamped ee_target_tf;


        for (int i = 0; i < 10; i++) {
            try {
                world_target_tf = tfBuffer.lookupTransform("world", part_in_camera_frame,
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose world_target{};
        world_target.position.x = world_target_tf.transform.translation.x;
        world_target.position.y = world_target_tf.transform.translation.y;
        world_target.position.z = world_target_tf.transform.translation.z;
        world_target.orientation.x = world_target_tf.transform.rotation.x;
        world_target.orientation.y = world_target_tf.transform.rotation.y;
        world_target.orientation.z = world_target_tf.transform.rotation.z;
        world_target.orientation.w = world_target_tf.transform.rotation.w;

        return world_target;
    }
    

    geometry_msgs::Pose transformtoWorldFrame(
        const geometry_msgs::Pose& target,
        std::string location) {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        std::string kit_tray;
        if (location.compare("agv1") == 0)
            kit_tray = "kit_tray_1";
        else if (location.compare("agv2") == 0)
            kit_tray = "kit_tray_2";
        else if (location.compare("agv3") == 0)
            kit_tray = "kit_tray_3";
        else if (location.compare("agv4") == 0)
            kit_tray = "kit_tray_4";
        else if (location.compare("as1") == 0)
            kit_tray = "briefcase_1";
        else if (location.compare("as2") == 0)
            kit_tray = "briefcase_2";
        else if (location.compare("as3") == 0)
            kit_tray = "briefcase_3";
        else if (location.compare("as4") == 0)
            kit_tray = "briefcase_4";

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = kit_tray;
        transformStamped.child_frame_id = "target_frame";
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;
      

        for (int i{ 0 }; i < 5; ++i)
            br.sendTransform(transformStamped);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);

        geometry_msgs::TransformStamped world_pose_tf;
        geometry_msgs::TransformStamped ee_target_tf;


        for (int i = 0; i < 10; i++) {
            try {
                world_pose_tf = tfBuffer.lookupTransform("world", "target_frame",
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        geometry_msgs::Pose world_pose{};
        world_pose.position.x = world_pose_tf.transform.translation.x;
        world_pose.position.y = world_pose_tf.transform.translation.y;
        world_pose.position.z = world_pose_tf.transform.translation.z;
        world_pose.orientation.x = world_pose_tf.transform.rotation.x;
        world_pose.orientation.y = world_pose_tf.transform.rotation.y;
        world_pose.orientation.z = world_pose_tf.transform.rotation.z;
        world_pose.orientation.w = world_pose_tf.transform.rotation.w;

        return world_pose;
    }

    geometry_msgs::Pose gettransforminWorldFrame(
        const geometry_msgs::Pose& target,
        std::string frame) {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        std::string header;
        std::string child;
        std::string child_frame;
        child = "target_";

        if (frame.compare("agv1") == 0)
            header = "kit_tray_1";
        else if (frame.compare("agv2") == 0)
            header = "kit_tray_2";
        else if (frame.compare("agv3") == 0)
            header = "kit_tray_3";
        else if (frame.compare("agv4") == 0)
            header = "kit_tray_4";
        else if (frame.compare("quality_control_sensor_1") == 0){
            header = "quality_control_sensor_1_frame";
            child = "quality_control_sensor_1_model_";
        }
        else if (frame.compare("quality_control_sensor_2") == 0){
            header = "quality_control_sensor_2_frame";
            child = "quality_control_sensor_2_model_";
        }
        else if (frame.compare("quality_control_sensor_3") == 0){
            header = "quality_control_sensor_3_frame";
            child = "quality_control_sensor_3_model_";
        }
        else if (frame.compare("quality_control_sensor_4") == 0){
            header = "quality_control_sensor_4_frame";
            child = "quality_control_sensor_4_model_";
        }
        else if (frame.compare("logical_camera_bins0") == 0){
            header = "logical_camera_bins0_frame";
            child = "logical_camera_bins0_model_";
        }
        else if (frame.compare("logical_camera_bins1") == 0){
            header = "logical_camera_bins1_frame";
            child = "logical_camera_bins1_model_";
        }
        else if (frame.compare("logical_camera_agv1as1") == 0){
            header = "logical_camera_agv1as1_frame";
            child = "logical_camera_agv1as1_model_";
        }
        else if (frame.compare("logical_camera_agv1as2") == 0){
            header = "logical_camera_agv1as2_frame";
            child = "logical_camera_agv1as2_model_";
        }
        else if (frame.compare("logical_camera_agv2as1") == 0){
            header = "logical_camera_agv2as1_frame";
            child = "logical_camera_agv2as1_model_";
        }
        else if (frame.compare("logical_camera_agv2as2") == 0){
            header = "logical_camera_agv2as2_frame";
            child = "logical_camera_agv2as2_model_";
        }
        else if (frame.compare("logical_camera_agv3as3") == 0){
            header = "logical_camera_agv3as3_frame";
            child = "logical_camera_agv3as3_model_";
        }
        else if (frame.compare("logical_camera_agv3as4") == 0){
            header = "logical_camera_agv3as4_frame";
            child = "logical_camera_agv3as4_model_";
        }
        else if (frame.compare("logical_camera_agv4as3") == 0){
            header = "logical_camera_agv4as3_frame";
            child = "logical_camera_agv4as3_model_";
        }
        else if (frame.compare("logical_camera_agv4as4") == 0){
            header = "logical_camera_agv4as4_frame";
            child = "logical_camera_agv4as4_model_";
        }
        else if (frame.compare("logical_camera_station1") == 0){
            header = "logical_camera_station1_frame";
            child = "logical_camera_station1_model_";
        }
        else if (frame.compare("logical_camera_station2") == 0){
            header = "logical_camera_station2_frame";
            child = "logical_camera_station2_model_";
        }
        else if (frame.compare("logical_camera_station3") == 0){
            header = "logical_camera_station3_frame";
            child = "logical_camera_station3_model_";
        }
        else if (frame.compare("logical_camera_station4") == 0){
            header = "logical_camera_station4_frame";
            child = "logical_camera_station4_model_";
        }
        

        child_frame = child + std::to_string(rand()) + "_frame";


        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = header;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;
      

        for (int i{ 0 }; i < 5; ++i)
            br.sendTransform(transformStamped);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);

        geometry_msgs::TransformStamped world_pose_tf;
        geometry_msgs::TransformStamped ee_target_tf;


        for (int i = 0; i < 10; i++) {
            try {
                world_pose_tf = tfBuffer.lookupTransform("world", child_frame,
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        geometry_msgs::Pose world_pose{};
        world_pose.position.x = world_pose_tf.transform.translation.x;
        world_pose.position.y = world_pose_tf.transform.translation.y;
        world_pose.position.z = world_pose_tf.transform.translation.z;
        world_pose.orientation.x = world_pose_tf.transform.rotation.x;
        world_pose.orientation.y = world_pose_tf.transform.rotation.y;
        world_pose.orientation.z = world_pose_tf.transform.rotation.z;
        world_pose.orientation.w = world_pose_tf.transform.rotation.w;

        return world_pose;
    }

    int get_empty_bin(std::vector<int> empty_bins){
        int bin_selected = 0;
        for(auto &bin: empty_bins){
            ROS_INFO_STREAM("bin number "<< bin);
            if(bin == 1 || bin == 2 || bin == 5 || bin == 6)
            {
                bin_selected = bin;
                break;
            }
        }
        
        return bin_selected;
    }

}