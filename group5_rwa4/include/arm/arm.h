#ifndef __ARM_H__
#define __ARM_H__

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
// custom
#include "../util/util.h"
#include "../comp/comp_class.h"

namespace motioncontrol {

    /**
     * @brief class for the Gantry robot
     *
     */
    class Arm {
        public:
        /**
        * @brief Struct for preset locations
        * @todo Add new preset locations here if needed
        */
        typedef struct ArmPresetLocation {
            std::vector<double> arm_preset;  //9 joints
            std::string name;
        } start, bin, agv, grasp, conveyor;

        Arm(ros::NodeHandle& node_handle);
        /**
         * @brief Initialize the object
         */
        void init();
        /**
         * @brief Pick part using kitting arm
         * 
         * @param part_type Type of part
         * @param part_pose Pose of the part in world
         * @return true 
         * @return false 
         */
        bool pickPart(std::string part_type, geometry_msgs::Pose part_pose);
        /**
         * @brief Pick faulty part from the agv
         * 
         * @param part_type Type of part
         * @param part_pose Pose of the part in world
         * @return true 
         * @return false 
         */
        bool pickfaulty(std::string part_type, geometry_msgs::Pose part_pose);
        /**
         * @brief Place the part on the agv
         * 
         * @param part_init_pose Initial pose of the part in world
         * @param part_goal_pose Target pose of the part in world
         * @param agv Agv_id
         * @return true 
         * @return false 
         */
        bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv);
        void testPreset(const std::vector<ArmPresetLocation>& preset_list);
        /**
         * @brief Pick and place part using kitting arm
         * 
         * @param part_type Type of part
         * @param pose_in_world_frame Initial pose of the part in world
         * @param goal_in_tray_frame Target pose of the part in world
         * @param agv Agv_id
         */
        void movePart(std::string part_type, geometry_msgs::Pose pose_in_world_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv);
        /**
         * @brief Activate kitting arm gripper
         * 
         */
        void activateGripper();
        /**
         * @brief Deactivate kitting arm gripper
         * 
         */
        void deactivateGripper();
        /**
         * @brief Get the pose of the part to be place in empty bin 
         * 
         * @param bin_number bin number, value in between 1-8
         * @return geometry_msgs::Pose Pose in world frame
         */
        geometry_msgs::Pose get_part_pose_in_empty_bin(int bin_number);
        /**
         * @brief Move the joint linear_arm_actuator_joint only
         *
         * The joint position for this joint corresponds to the y value
         * in the world frame. For instance, a value of 0 for this joint
         * moves the base of the robot to y = 0.
         *
         * @param location A preset location
         */
        void moveBaseTo(double linear_arm_actuator_joint_position);
        /**
         * @brief Get the Gripper State
         * 
         * @return nist_gear::VacuumGripperState 
         */
        nist_gear::VacuumGripperState getGripperState();

        

        /**
         * @brief Send command message to robot controller
         * 
         * @param command_msg 
         * @return true 
         * @return false 
         */
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        /**
         * @brief Move the kitting arm to preset location
         * 
         * @param location_name Location
         */
        void goToPresetLocation(std::string location_name);
        /**
         * @brief Pick part from conveyor
         * 
         * @param ebin empty bin number
         * @param int number of parts to be picked
         * @return std::vector<int> 
         */
        std::vector<int> pick_from_conveyor(std::vector<int> ebin, unsigned short int);
        /**
         * @brief Flips the part(pump)
         * 
         * @param part Product
         * @param rbin empty bins
         * @param part_pose_in_frame Pose in world frame 
         * @param agv Agv_id
         */
        void flippart(Product part, std::vector<int> rbin, geometry_msgs::Pose part_pose_in_frame, std::string agv, bool);

        //--preset locations;
        start home1_, home2_;
        agv agv1_, agv2_, agv3_, agv4_;
        conveyor on_, above_, flip_;

        private:
        std::array<double,3> bin1_origin_ { -1.898, 3.37, 0.751 };
        std::array<double,3> bin2_origin_ { -1.898, 2.56, 0.751 };
        std::array<double,3> bin3_origin_ { -2.651, 2.56, 0.751 };
        std::array<double,3> bin4_origin_ { -2.651, 3.37, 0.751 };
        std::array<double,3> bin5_origin_ { -1.898, -3.37, 0.751 };
        std::array<double,3> bin6_origin_ { -1.898, -2.56, 0.751 };
        std::array<double,3> bin7_origin_ { -2.651, -2.56, 0.751 };
        std::array<double,3> bin8_origin_ { -2.651, -3.37, 0.751 };
        int empty_bin_place_count = 0;
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options arm_options_;
        moveit::planning_interface::MoveGroupInterface arm_group_;
        sensor_msgs::JointState current_joint_states_;
        control_msgs::JointTrajectoryControllerState arm_controller_state_;

        nist_gear::VacuumGripperState gripper_state_;
        // gripper state subscriber
        ros::Subscriber gripper_state_subscriber_;
        // service client
        ros::ServiceClient gripper_control_client_;
        // publishers
        ros::Publisher arm_joint_trajectory_publisher_;
        // joint states subscribers
        ros::Subscriber arm_joint_states_subscriber_;
        // controller state subscribers
        ros::Subscriber arm_controller_state_subscriber_;

        // callbacks
        void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
        void arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    };
}//namespace


namespace gantry_motioncontrol {


    /**
     * @brief class for the Gantry robot
     *
     */
    class Gantry {

        public:
        /**
     * @brief Struct for preset locations
     * @todo Add new preset locations here if needed
     *
     */
        typedef struct GantryPresetLocation {
            std::vector<double> gantry_full_preset;  //3 joints
            std::vector<double> gantry_torso_preset; //6 joints
            std::vector<double> gantry_arm_preset;   //9 joints
        } start, bin, agv, grasp, near_as, as;

        Gantry(ros::NodeHandle& node);
        /**
         * @brief Initialize the object
         *
         * Set the different groups.
         *
         */
        void init();
        /**
         * @brief Picks the part using gantry arm
         * 
         * @param part_init_pose Initial pose in world
         * @return true 
         * @return false 
         */
        bool pickPart(geometry_msgs::Pose part_init_pose);
        /**
         * @brief Places the part
         * 
         * @param part_init_pose Initial pose in world
         * @param part_pose_in_frame Goal pose in world
         * @param agv Agv_id
         * @return true 
         * @return false 
         */
        bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame, std::string agv);
        /**
         * @brief Picks and places the part
         * 
         * @param part_init_pose_in_world Initial pose in world
         * @param target_pose_in_frame Goal pose in world
         * @param location Agv or assembly station id.
         * @param type Part type
         * @return true 
         * @return false 
         */
        bool movePart(geometry_msgs::Pose part_init_pose_in_world, geometry_msgs::Pose target_pose_in_frame, std::string location, std::string type);
        /**
         * @brief Picks and places part on empty bin
         * 
         * @param part_init_pose_in_world Initial pose in world
         * @param type Part type
         * @param bin bin number
         * @return true 
         * @return false 
         */
        bool movePartfrombin(geometry_msgs::Pose part_init_pose_in_world, std::string type, unsigned short int bin);
        /**
         * @brief Moves the gantry to preset location near bin
         * 
         * @param bin 
         */
        void move_gantry_to_bin(unsigned short int bin);
        /**
         * @brief Moves the gantry to preset location near assembly station
         * 
         * @param c_name 
         */
        void move_gantry_to_assembly_station(std::string c_name);
        /**
         * @brief Flips the part
         * 
         * @param part Product
         * @param rbin empty bin
         * @param part_pose_in_frame Initial pose in world
         * @param agv Agv_id
         */
        void flippart(Product part, std::vector<int> rbin, geometry_msgs::Pose part_pose_in_frame, std::string agv, bool);

        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(GantryPresetLocation location, bool full_robot=true);
        void activateGripper();
        void deactivateGripper();
        nist_gear::VacuumGripperState getGripperState();
        //--preset locations;
        start home_, home2_;
        bin at_bin1_,at_bin2_, at_bin3_, at_bin4_, at_bin5_, at_bin6_, at_bin7_, at_bin8_, safe_bins_, at_bins1234_, at_bins5678_;
        agv at_agv1_, at_agv2_, at_agv3_, at_agv4_, at_agv1_as1_, at_agv1_as2_, at_agv2_as1_, at_agv2_as2_, at_agv3_as3_, at_agv3_as4_, at_agv4_as3_, at_agv4_as4_;
        grasp pre_grasp_, post_grasp_;
        near_as near_as1_, near_as2_, near_as3_, near_as4_;
        as at_as1_, at_as2_, at_as3_, at_as4_;

        private:
        std::array<double,3> bin1_origin_ { -1.898, 3.37, 0.751 };
        std::array<double,3> bin2_origin_ { -1.898, 2.56, 0.751 };
        std::array<double,3> bin3_origin_ { -2.651, 2.56, 0.751 };
        std::array<double,3> bin4_origin_ { -2.651, 3.37, 0.751 };
        std::array<double,3> bin5_origin_ { -1.898, -3.37, 0.751 };
        std::array<double,3> bin6_origin_ { -1.898, -2.56, 0.751 };
        std::array<double,3> bin7_origin_ { -2.651, -2.56, 0.751 };
        std::array<double,3> bin8_origin_ { -2.651, -3.37, 0.751 };
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options full_gantry_options_;
        moveit::planning_interface::MoveGroupInterface::Options arm_gantry_options_;
        moveit::planning_interface::MoveGroupInterface::Options torso_gantry_options_;
        moveit::planning_interface::MoveGroupInterface full_gantry_group_;
        moveit::planning_interface::MoveGroupInterface arm_gantry_group_;
        moveit::planning_interface::MoveGroupInterface torso_gantry_group_;
        sensor_msgs::JointState current_joint_states_;
        nist_gear::VacuumGripperState gantry_gripper_state_;
        control_msgs::JointTrajectoryControllerState gantry_torso_controller_state_;
        control_msgs::JointTrajectoryControllerState gantry_arm_controller_state_;

        // publishers
        ros::Publisher gantry_torso_joint_trajectory_publisher_;
        ros::Publisher gantry_arm_joint_trajectory_publisher_;

        // joint states subscribers
        ros::Subscriber gantry_full_joint_states_subscriber_;
        // gripper state subscriber
        ros::Subscriber gantry_gripper_state_subscriber_;
        // controller state subscribers
        ros::Subscriber gantry_controller_state_subscriber_;
        ros::Subscriber gantry_arm_controller_state_subscriber_;

        // service client
        ros::ServiceClient gantry_gripper_control_client_;

        // For visualizing things in rviz
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

        // callbacks
        void gantry_full_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void gantry_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& msg);
        void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
        void gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    };
} //namespace

#endif
