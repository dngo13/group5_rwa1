#include "../include/arm/arm.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>
#include "../include/util/util.h"
#include <math.h>

namespace motioncontrol {
    /////////////////////////////////////////////////////
    Arm::Arm(ros::NodeHandle& node) : node_("/ariac/kitting"),
        planning_group_("/ariac/kitting/robot_description"),
        arm_options_("kitting_arm", planning_group_, node_),
        arm_group_(arm_options_)
    {
        ROS_INFO_STREAM("[Arm] constructor called... ");

    }

    /////////////////////////////////////////////////////
    void Arm::init()
    {
        // make sure the planning group operates in the world frame
        // check the name of the end effector
        // ROS_INFO_NAMED("init", "End effector link: %s", arm_group_.getEndEffectorLink().c_str());


        // publishers to directly control the joints without moveit
        arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);
        // joint state subscribers
        arm_joint_states_subscriber_ =
            node_.subscribe("/ariac/kitting/joint_states", 10, &Arm::arm_joint_states_callback_, this);
        // controller state subscribers
        arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/kitting/kitting_arm_controller/state", 10, &Arm::arm_controller_state_callback, this);
        // gripper state subscriber
        gripper_state_subscriber_ = node_.subscribe(
            "/ariac/kitting/arm/gripper/state", 10, &Arm::gripper_state_callback, this);
        // controller state subscribers
        gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
        gripper_control_client_.waitForExistence();


        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the arm are in this order:
        // - linear_arm_actuator_joint
        // - shoulder_pan_joint
        // - shoulder_lift_joint
        // - elbow_joint
        // - wrist_1_joint
        // - wrist_2_joint
        // - wrist_3_joint

        double linear_arm_actuator_joint{ 0 };
        double shoulder_pan_joint{ 0 };
        double shoulder_lift_joint{ -1.25 };
        double elbow_joint{ 1.74 };
        double wrist_1_joint{ -2.06 };
        double wrist_2_joint{ -1.51 };
        double wrist_3_joint{ 0 };

        //home position
        home1_.arm_preset = { 0, 0, -1.25, 1.74, -2.04, -1.57, 0 };
        home1_.name = "home1";
        home2_.arm_preset = { 0, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        home2_.name = "home2";
        on_.arm_preset = { 1.76 , 0, -0.74, 1.76, 5.28, 0, 0 };
        on_.name = "on"; 
        above_.arm_preset = { 1.76 , 0, -1.62, 1.76, 6.28, 0, 0.0 };
        above_.name = "above";
        agv1_.arm_preset = { 3.83, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv1_.name = "agv1";
        agv2_.arm_preset = { 0.83, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv2_.name = "agv2";
        agv3_.arm_preset = { -1.83, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv3_.name = "agv3";
        agv4_.arm_preset = { -4.33, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv4_.name = "agv4";
        flip_.arm_preset = { -3.97, 0, -2.54, -2.0, 4.56, 0, 0};
        flip_.name = "flip";
        // bin1_.arm_preset = { 3.2 , 1.51 , -1.12 , 1.76, -2.04, -1.57, 0 };
        // bin1_.name = "bin1";




        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();
        // next get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);
    }

    //////////////////////////////////////////////////////
    void Arm::moveBaseTo(double linear_arm_actuator_joint_position) {
        // get the current joint positions
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();

        // get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

        // next, assign a value to only the linear_arm_actuator_joint
        joint_group_positions_.at(0) = linear_arm_actuator_joint_position;

        // move the arm
        arm_group_.setJointValueTarget(joint_group_positions_);
        arm_group_.move();
    }
    //////////////////////////////////////////////////////
    void Arm::movePart(std::string part_type, geometry_msgs::Pose pose_in_world_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv) {
        //convert goal_in_tray_frame into world frame
        // auto init_pose_in_world = motioncontrol::transformToWorldFrame(camera_frame);
        auto init_pose_in_world = pose_in_world_frame;

        // ROS_INFO_STREAM(init_pose_in_world.position.x << " " << init_pose_in_world.position.y);
        // auto target_pose_in_world = motioncontrol::transformtoWorldFrame(goal_in_tray_frame, agv);
        auto target_pose_in_world = motioncontrol::gettransforminWorldFrame(goal_in_tray_frame, agv);
        
        if (pickPart(part_type, init_pose_in_world)) {
            placePart(init_pose_in_world, goal_in_tray_frame, agv);
        }
    }
    /////////////////////////////////////////////////////
    nist_gear::VacuumGripperState Arm::getGripperState()
    {
        return gripper_state_;
    }

    /**
     * @brief Pick up a part from a bin
     *
     * @param part Part to pick up
     * @return true Part was picked up
     * @return false Part was not picked up
     *
     * We use the group full_gantry_group_ to allow the robot more flexibility
     */
    bool Arm::pickPart(std::string part_type, geometry_msgs::Pose part_init_pose) {
        arm_group_.setMaxVelocityScalingFactor(1.0);
        moveBaseTo(part_init_pose.position.y - 0.3);
        ROS_INFO_STREAM("z of part: " << part_init_pose.position.z);
        // // move the arm above the part to grasp
        // // gripper stays at the current z
        // // only modify its x and y based on the part to grasp
        // // In this case we do not need to use preset locations
        // // everything is done dynamically

        // Make sure the wrist is facing down
        // otherwise it will have a hard time attaching a part
        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();
        
        // post-grasp pose 3
        // store the pose of the arm before it goes down to pick the part
        // we will bring the arm back to this pose after picking up the part
        auto postgrasp_pose3 = part_init_pose;
        postgrasp_pose3.orientation = arm_ee_link_pose.orientation;
        postgrasp_pose3.position.z = arm_ee_link_pose.position.z + 0.25;

        // preset z depending on the part type
        // some parts are bigger than others
        
        double z_pos{};
        if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.83;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.79;
        }
        if (part_type.find("regulator") != std::string::npos) {
            z_pos = 0.80;
        }
        if (part_type.find("battery") != std::string::npos) {
            z_pos = 0.78;
        }
        
        // set of waypoints the arm will go through
        std::vector<geometry_msgs::Pose> waypoints;
        // pre-grasp pose: somewhere above the part
        auto pregrasp_pose = part_init_pose;
        pregrasp_pose.orientation = arm_ee_link_pose.orientation;
        pregrasp_pose.position.z = z_pos + 0.06;

        // grasp pose: right above the part
        auto grasp_pose = part_init_pose;
        grasp_pose.orientation = arm_ee_link_pose.orientation;
        grasp_pose.position.z = z_pos + 0.03;

        waypoints.push_back(pregrasp_pose);
        waypoints.push_back(grasp_pose);

        // activate gripper
        // sometimes it does not activate right away
        // so we are doing this in a loop
        while (!gripper_state_.enabled) {
            activateGripper();
        }

        // move the arm to the pregrasp pose
        arm_group_.setPoseTarget(pregrasp_pose);
        arm_group_.move();

        
        /* Cartesian motions are frequently needed to be slower for actions such as approach
        and retreat grasp motions. Here we demonstrate how to reduce the speed and the acceleration
        of the robot arm via a scaling factor of the maxiumum speed of each joint.
        */
        arm_group_.setMaxVelocityScalingFactor(0.05);
        arm_group_.setMaxAccelerationScalingFactor(0.05);
        // plan the cartesian motion and execute it
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm_group_.execute(plan);

        ros::Duration(sleep(2.0));

        // move the arm 1 mm down until the part is attached
        while (!gripper_state_.attached) {
            grasp_pose.position.z -= 0.001;
            arm_group_.setPoseTarget(grasp_pose);
            arm_group_.move();
            ros::Duration(sleep(0.5));
        }
        
            arm_group_.setMaxVelocityScalingFactor(1.0);
            arm_group_.setMaxAccelerationScalingFactor(1.0);
            ROS_INFO_STREAM("[Gripper] = object attached");
            ros::Duration(sleep(2.0));
            arm_group_.setPoseTarget(postgrasp_pose3);
            arm_group_.move();

            return true;
        
    }

    bool Arm::pickfaulty(std::string part_type, geometry_msgs::Pose part_init_pose) {
        arm_group_.setMaxVelocityScalingFactor(1.0);
        moveBaseTo(part_init_pose.position.y - 0.3);
        ROS_INFO_STREAM("z of part: " << part_init_pose.position.z);
        // // move the arm above the part to grasp
        // // gripper stays at the current z
        // // only modify its x and y based on the part to grasp
        // // In this case we do not need to use preset locations
        // // everything is done dynamically

        // Make sure the wrist is facing down
        // otherwise it will have a hard time attaching a part
        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();
        arm_group_.setMaxVelocityScalingFactor(1);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        // post-grasp pose 3
        // store the pose of the arm before it goes down to pick the part
        // we will bring the arm back to this pose after picking up the part
        // auto postgrasp_pose3 = part_init_pose;
        // postgrasp_pose3.orientation = arm_ee_link_pose.orientation;
        // postgrasp_pose3.position.z += 0.05;

        // preset z depending on the part type
        // some parts are bigger than others
        
        double z_pos{};
        if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.83;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.79;
        }
        if (part_type.find("regulator") != std::string::npos) {
            z_pos = 0.80;
        }
        if (part_type.find("battery") != std::string::npos) {
            z_pos = 0.78;
        }

        arm_ee_link_pose.position.x = part_init_pose.position.x;
        arm_ee_link_pose.position.y = part_init_pose.position.y;
        arm_ee_link_pose.position.z = part_init_pose.position.z + 0.2;
        ROS_INFO_STREAM("EE_Z " <<arm_ee_link_pose.position.z);
        arm_group_.setMaxVelocityScalingFactor(1);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        ros::Duration(sleep(0.5));
        
        arm_ee_link_pose.position.z = arm_ee_link_pose.position.z - 0.1;
        ROS_INFO_STREAM("EE_Z " <<arm_ee_link_pose.position.z);
        arm_group_.setMaxVelocityScalingFactor(1);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        ros::Duration(sleep(0.5));

        // // activate gripper
        // // sometimes it does not activate right away
        // // so we are doing this in a loop
        while (!gripper_state_.enabled) {
            activateGripper();
        }


        
        /* Cartesian motions are frequently needed to be slower for actions such as approach
        and retreat grasp motions. Here we demonstrate how to reduce the speed and the acceleration
        of the robot arm via a scaling factor of the maxiumum speed of each joint.
        */

        // move the arm 1 mm down until the part is attached
        while (!gripper_state_.attached) {
            arm_ee_link_pose.position.z = arm_ee_link_pose.position.z - 0.001;
            ROS_INFO_STREAM("EE_Z in loop " <<arm_ee_link_pose.position.z);
            arm_group_.setMaxVelocityScalingFactor(1);
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            ros::Duration(sleep(0.5));
        }
            arm_ee_link_pose = arm_group_.getCurrentPose().pose;
             arm_ee_link_pose.position.z = arm_ee_link_pose.position.z + 0.5;
            arm_group_.setMaxVelocityScalingFactor(1.0);
            // arm_group_.setMaxAccelerationScalingFactor(1.0);
            ROS_INFO_STREAM("[Gripper] = object attached");
            ros::Duration(sleep(2.0));
            // geometry_msgs::Pose arm_ee_link_pose1 = arm_group_.getCurrentPose().pose;
            // arm_ee_link_pose.position.z += 0.5;
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();

            return true;
        
    }
    /////////////////////////////////////////////////////
    bool Arm::placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame, std::string agv)
    {
        goToPresetLocation(agv);
        // get the target pose of the part in the world frame
        auto target_pose_in_world = motioncontrol::transformtoWorldFrame(
            part_pose_in_frame,
            agv);

        // moveBaseTo(target_pose_in_world.position.y - 0.1);
        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();

        // store the current orientation of the end effector now
        // so we can reuse it later
        tf2::Quaternion q_current(
            arm_ee_link_pose.orientation.x,
            arm_ee_link_pose.orientation.y,
            arm_ee_link_pose.orientation.z,
            arm_ee_link_pose.orientation.w);
        
        // move the arm above the agv
        // gripper stays at the current z
        // only modify its x and y based on the part to grasp
        // In this case we do not need to use preset locations
        // everything is done dynamically
        arm_ee_link_pose.position.x = target_pose_in_world.position.x;
        arm_ee_link_pose.position.y = target_pose_in_world.position.y;
        // move the arm
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();

        


        // orientation of the part in the bin, in world frame
        tf2::Quaternion q_init_part(
            part_init_pose.orientation.x,
            part_init_pose.orientation.y,
            part_init_pose.orientation.z,
            part_init_pose.orientation.w);
        // orientation of the part in the tray, in world frame
        tf2::Quaternion q_target_part(
            target_pose_in_world.orientation.x,
            target_pose_in_world.orientation.y,
            target_pose_in_world.orientation.z,
            target_pose_in_world.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        // orientation of the gripper when placing the part in the tray
        target_pose_in_world.orientation.x = q_rslt.x();
        target_pose_in_world.orientation.y = q_rslt.y();
        target_pose_in_world.orientation.z = q_rslt.z();
        target_pose_in_world.orientation.w = q_rslt.w();
        target_pose_in_world.position.z += 0.15;

        arm_group_.setMaxVelocityScalingFactor(0.1);
        arm_group_.setPoseTarget(target_pose_in_world);
        arm_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();

        arm_group_.setMaxVelocityScalingFactor(1.0);
        goToPresetLocation("home2");

        return true;
        
    }
    /////////////////////////////////////////////////////
    void Arm::gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
    {
        gripper_state_ = *gripper_state_msg;
    }
    /////////////////////////////////////////////////////
    void Arm::activateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Arm][activateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Arm::deactivateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = false;
        gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Arm][deactivateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Arm::goToPresetLocation(std::string location_name)
    {

        ArmPresetLocation location;
        if (location_name.compare("home1") == 0) {
            location = home1_;
        }
        else if (location_name.compare("home2") == 0) {
            location = home2_;
        }
        else if (location_name.compare("on") == 0) {
            location = on_;
        }
        else if (location_name.compare("above") == 0) {
            location = above_;
        }
        else if (location_name.compare("agv1") == 0) {
            location = agv1_;
        }
        else if (location_name.compare("agv2") == 0) {
            location = agv2_;
        }
        else if (location_name.compare("agv3") == 0) {
            location = agv3_;
        }
        else if (location_name.compare("agv4") == 0) {
            location = agv4_;
        }
        else if (location_name.compare("flip") == 0) {
            location = flip_;
        }
        joint_group_positions_.at(0) = location.arm_preset.at(0);
        joint_group_positions_.at(1) = location.arm_preset.at(1);
        joint_group_positions_.at(2) = location.arm_preset.at(2);
        joint_group_positions_.at(3) = location.arm_preset.at(3);
        joint_group_positions_.at(4) = location.arm_preset.at(4);
        joint_group_positions_.at(5) = location.arm_preset.at(5);
        joint_group_positions_.at(6) = location.arm_preset.at(6);

        arm_group_.setJointValueTarget(joint_group_positions_);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // check a plan is found first then execute the action
        bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            arm_group_.move();
    }

    geometry_msgs::Pose Arm::get_part_pose_in_empty_bin(int bin_number){
        std::array<double,3> bin_origin{0,0,0};
        geometry_msgs::Pose part_world_pose;
        ROS_INFO_STREAM("empty_bin_place_count "<<empty_bin_place_count);
        if (bin_number == 1){
            bin_origin = bin1_origin_;
        }
        if (bin_number == 2){
            bin_origin = bin2_origin_;
        }
        if (bin_number == 3){
            bin_origin = bin3_origin_;
        }
        if (bin_number == 4){
            bin_origin = bin4_origin_;
        }
        if (bin_number == 5){
            bin_origin = bin5_origin_;
        }
        if (bin_number == 6){
            bin_origin = bin6_origin_;
        }
        if (bin_number == 7){
            bin_origin = bin7_origin_;
        }
        if (bin_number == 8){
            bin_origin = bin8_origin_;
        }

        if (empty_bin_place_count == 0){
            part_world_pose.position.x = bin_origin.at(0) + 0.20;
            part_world_pose.position.y = bin_origin.at(1) - 0.15;
            part_world_pose.position.z = bin_origin.at(2);
            empty_bin_place_count++;
        }
        else if (empty_bin_place_count == 1){
            part_world_pose.position.x = bin_origin.at(0) + 0.20;
            part_world_pose.position.y = bin_origin.at(1) + 0.15;
            part_world_pose.position.z = bin_origin.at(2);
            empty_bin_place_count++;
        }
        else if (empty_bin_place_count == 2){
            part_world_pose.position.x = bin_origin.at(0) - 0.10;
            part_world_pose.position.y = bin_origin.at(1) + 0.15;
            part_world_pose.position.z = bin_origin.at(2);
            empty_bin_place_count++;
        }
        else if (empty_bin_place_count == 3){
            part_world_pose.position.x = bin_origin.at(0) - 0.10;
            part_world_pose.position.y = bin_origin.at(1) - 0.15;
            part_world_pose.position.z = bin_origin.at(2);
        }
        return part_world_pose;
    }

    std::vector<int>  Arm::pick_from_conveyor(std::vector<int> empty_bins_at_start, unsigned short int n)
    {   
        std::vector<int> empty_bins;
        int bin_selected = 0;
        for(auto &bin: empty_bins_at_start){
            ROS_INFO_STREAM("bin number "<< bin);
            if(bin == 1 || bin == 2 || bin == 5 || bin == 6)
            {
                bin_selected = bin;
                break;
            }
        }

        for(auto &bin: empty_bins_at_start){
            if(bin == bin_selected){
                continue;
            }
            else{
                empty_bins.push_back(bin);
            }
        }
        for(int i = 0 ; i < n; i++){
            double trigger_time_ = ros::Time::now().toSec();
            
            goToPresetLocation("on");
            geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
            auto side_orientation = motioncontrol::quaternionFromEuler(0, 0, 1.57);
            while (!gripper_state_.enabled) {
                activateGripper();
            }
            while (!gripper_state_.attached && ros::Time::now().toSec() - trigger_time_ < 15){
                
            }
            ROS_INFO_STREAM("object attached"); 
            // arm_ee_link_pose.position.z = arm_ee_link_pose.position.z + 0.009;
            side_orientation = motioncontrol::quaternionFromEuler(0, 0, 0);
            arm_ee_link_pose = arm_group_.getCurrentPose().pose;
            arm_ee_link_pose.position.z = arm_ee_link_pose.position.z + 0.5; 
            arm_ee_link_pose.orientation.x = side_orientation.getX();
            arm_ee_link_pose.orientation.y = side_orientation.getY();
            arm_ee_link_pose.orientation.z = side_orientation.getZ();
            arm_ee_link_pose.orientation.w = side_orientation.getW();
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            
            ROS_INFO_STREAM("Selected bin number "<< bin_selected);
            geometry_msgs::Pose bin = get_part_pose_in_empty_bin(bin_selected);
            ROS_INFO_STREAM("Y_pos: "<< bin.position.y);
            side_orientation = motioncontrol::quaternionFromEuler(0, 0, 0);
            moveBaseTo(bin.position.y);
            arm_ee_link_pose.position.x = bin.position.x - 0.15;
            arm_ee_link_pose.position.y = bin.position.y;
            arm_ee_link_pose.position.z = bin.position.z + 0.5;
            arm_group_.setMaxVelocityScalingFactor(1.0);
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            arm_ee_link_pose.position.z = bin.position.z + 0.1;
            arm_group_.setMaxVelocityScalingFactor(1.0);
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            // get the current joint positions
            const moveit::core::JointModelGroup* joint_model_group =
                arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
            moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();

            // get the current set of joint values for the group.
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

            // next, assign a value to only the linear_arm_actuator_joint
            joint_group_positions_.at(6) = M_PI / 9;

            // move the arm
            arm_group_.setJointValueTarget(joint_group_positions_);
            arm_group_.move();


            ros::Duration(2.0).sleep();
            deactivateGripper();
            goToPresetLocation("above");
        }
        // goToPresetLocation(bin);
        return empty_bins;
    }
    ///////////////////////////////
    void Arm::flippart(Product part, std::vector<int> empty_bins, geometry_msgs::Pose part_pose_in_frame, std::string agv, bool arm_required){
        std::string part_type = part.type;
        geometry_msgs::Pose part_pose = part.world_pose;
        ROS_INFO_STREAM("In flip");
        
        int bin_selected = 0;
        for(auto &bin: empty_bins){
            ROS_INFO_STREAM("bin number "<< bin);
            if(bin == 1 || bin == 2 || bin == 5 || bin == 6)
            {
                bin_selected = bin;
                break;
            }
        }
        if(bin_selected == 0){
            bin_selected = 2;
        }
        
        std::array<double,3> bin_origin{0,0,0};
        geometry_msgs::Pose part_world_pose;
        if (bin_selected == 1){
            bin_origin = bin1_origin_;
        }
        if (bin_selected == 2){
            bin_origin = bin2_origin_;
        }
        if (bin_selected == 3){
            bin_origin = bin3_origin_;
        }
        if (bin_selected == 4){
            bin_origin = bin4_origin_;
        }
        if (bin_selected == 5){
            bin_origin = bin5_origin_;
        }
        if (bin_selected == 6){
            bin_origin = bin6_origin_;
        }
        if (bin_selected == 7){
            bin_origin = bin7_origin_;
        }
        if (bin_selected == 8){
            bin_origin = bin8_origin_;
        }
        // ROS_INFO_STREAM("EMPTYBIN: "<<bin_selected);

        if (arm_required){
        pickPart(part_type, part_pose);
        }         
        moveBaseTo(bin_origin.at(1)-0.8);
        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();
        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1)-0.25 ;
        arm_ee_link_pose.position.z = bin_origin.at(2)+0.15;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        
        tf2::Quaternion q_current(
            arm_ee_link_pose.orientation.x,
            arm_ee_link_pose.orientation.y,
            arm_ee_link_pose.orientation.z,
            arm_ee_link_pose.orientation.w);
        auto target_pose = motioncontrol::quaternionFromEuler(0, 0, -1.57);
        tf2::Quaternion q_init_part(
            part_pose.orientation.x,
            part_pose.orientation.y,
            part_pose.orientation.z,
            part_pose.orientation.w);
        tf2::Quaternion q_target_part(
            target_pose.getX(),
            target_pose.getY(),
            target_pose.getZ(),
            target_pose.getW());
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();
        // orientation of the gripper when placing the part in the tray
        arm_ee_link_pose.orientation.x = q_rslt.x();
        arm_ee_link_pose.orientation.y = q_rslt.y();
        arm_ee_link_pose.orientation.z = q_rslt.z();
        arm_ee_link_pose.orientation.w = q_rslt.w();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        ros::Duration(2.0).sleep();
        // moveBaseTo(bin_origin.at(1)-0.4);
        part_pose.position.x = arm_ee_link_pose.position.x;
        part_pose.position.y = arm_ee_link_pose.position.y;
        part_pose.position.z = 0.8;
        part_pose.orientation.x = target_pose.getX();
        part_pose.orientation.y = target_pose.getY();
        part_pose.orientation.z = target_pose.getZ();
        part_pose.orientation.w = target_pose.getW();
        moveBaseTo(bin_origin.at(1)-0.6);
        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1);
        arm_ee_link_pose.position.z = bin_origin.at(2) + 0.3;
        geometry_msgs::Pose Post_grasp = arm_ee_link_pose;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        // goToPresetLocation("flip");
        auto side_orientation = motioncontrol::quaternionFromEuler(0, 0, -1.57);
        arm_ee_link_pose.orientation.x = side_orientation.getX();
        arm_ee_link_pose.orientation.y = side_orientation.getY();
        arm_ee_link_pose.orientation.z = side_orientation.getZ();
        arm_ee_link_pose.orientation.w = side_orientation.getW();
        // target_pose.position.z = bin_origin.at(2)+0.2;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        
        while (!gripper_state_.enabled) {
            activateGripper();
        }
        
        arm_ee_link_pose.position.x = part_pose.position.x;
        arm_ee_link_pose.position.y = part_pose.position.y+0.12 ;
        arm_ee_link_pose.position.z = part_pose.position.z;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        

        while (!gripper_state_.attached) {
            arm_ee_link_pose.position.y -= 0.005;
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            ros::Duration(sleep(0.5));
        }

        arm_ee_link_pose.position.z =arm_ee_link_pose.position.z+0.15;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        
        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1)+0.07;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();

        // get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

        // next, assign a value to only the linear_arm_actuator_joint
        ROS_INFO_STREAM("wrist3 "<<joint_group_positions_.at(6));
        joint_group_positions_.at(6) = joint_group_positions_.at(6) + M_PI;
        // move the arm
        ROS_INFO_STREAM("wrist3 "<<joint_group_positions_.at(6));
        arm_group_.setJointValueTarget(joint_group_positions_);
        arm_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        part.world_pose.position.x = bin_origin.at(0);
        part.world_pose.position.y = bin_origin.at(1);
        part.world_pose.position.z = 0.8;
        auto final_orientation = motioncontrol::quaternionFromEuler(3.14, 0, -1.57);
        part.world_pose.orientation.x = final_orientation.getX();
        part.world_pose.orientation.y = final_orientation.getY();
        part.world_pose.orientation.z = final_orientation.getZ();
        part.world_pose.orientation.w = final_orientation.getW();
        goToPresetLocation("home2");
        movePart(part_type,part.world_pose,part_pose_in_frame, agv);

    }

    ///////////////////////////
    ////// Callback Functions
    ///////////////////////////

    /////////////////////////////////////////////////////
    void Arm::arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
    {
        if (joint_state_msg->position.size() == 0) {
            ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
        }
        current_joint_states_ = *joint_state_msg;
    }

    /////////////////////////////////////////////////////
    void Arm::arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        arm_controller_state_ = *msg;
    }
}//namespace


namespace gantry_motioncontrol {
    /////////////////////////////////////////////////////
    Gantry::Gantry(ros::NodeHandle& node) : node_("/ariac/gantry"),
        planning_group_("/ariac/gantry/robot_description"),
        full_gantry_options_("gantry_full", planning_group_, node_),
        arm_gantry_options_("gantry_arm", planning_group_, node_),
        torso_gantry_options_("gantry_torso", planning_group_, node_),
        full_gantry_group_(full_gantry_options_),
        arm_gantry_group_(arm_gantry_options_),
        torso_gantry_group_(torso_gantry_options_)
    {
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers"));
        ROS_INFO_STREAM("[Gantry] constructor called... ");
    }



    /////////////////////////////////////////////////////
    void Gantry::init()
    {


        // publishers to directly control the joints without moveit
        gantry_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);
        gantry_torso_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

        // joint state subscribers
        gantry_full_joint_states_subscriber_ =
            node_.subscribe("/ariac/gantry/joint_states", 10, &Gantry::gantry_full_joint_states_callback_, this);
        // gripper state subscriber
        gantry_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/arm/gripper/state", 10, &Gantry::gantry_gripper_state_callback, this);
        // controller state subscribers
        gantry_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &Gantry::gantry_controller_state_callback, this);
        gantry_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_arm_controller/state", 10, &Gantry::gantry_arm_controller_state_callback, this);

        gantry_gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
        gantry_gripper_control_client_.waitForExistence();

        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the gantry are in this order:
        // For the torso
        // - small_long_joint
        // - torso_rail_joint
        // - torso_base_main_joint
        // For the arm
        // - gantry_arm_shoulder_pan_joint
        // - gantry_arm_shoulder_lift_joint
        // - gantry_arm_elbow_joint
        // - gantry_arm_wrist_1
        // - gantry_arm_wrist_2
        // - gantry_arm_wrist_3
        // For the full robot = torso + arm

 
        home_.gantry_torso_preset = { -3.3, 0.0, -1.57 };
        home_.gantry_arm_preset = {-0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        //concatenate gantry torso and gantry arm
        home_.gantry_full_preset.insert(home_.gantry_full_preset.begin(), home_.gantry_torso_preset.begin(), home_.gantry_torso_preset.end());
        home_.gantry_full_preset.insert(home_.gantry_full_preset.end(), home_.gantry_arm_preset.begin(), home_.gantry_arm_preset.end());
        // print(home_.gantry_full);

        home2_.gantry_torso_preset = { -8.3, 0.0, -1.57 };
        home2_.gantry_arm_preset = {-0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        home2_.gantry_full_preset.insert(home2_.gantry_full_preset.begin(), home2_.gantry_torso_preset.begin(), home2_.gantry_torso_preset.end());
        home2_.gantry_full_preset.insert(home2_.gantry_full_preset.end(), home2_.gantry_arm_preset.begin(), home2_.gantry_arm_preset.end());

        //safe spot to reach any bin without colliding with anything
        safe_bins_.gantry_torso_preset = { -6.90, -0.13, -0.02 };
        safe_bins_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        safe_bins_.gantry_full_preset.insert(safe_bins_.gantry_full_preset.begin(), safe_bins_.gantry_torso_preset.begin(), safe_bins_.gantry_torso_preset.end());
        safe_bins_.gantry_full_preset.insert(safe_bins_.gantry_full_preset.end(), safe_bins_.gantry_arm_preset.begin(), safe_bins_.gantry_arm_preset.end());

        //at bins 1, 2, 3, 4
        at_bins1234_.gantry_torso_preset = { -1.72, -2.90, -1.57 };
        at_bins1234_.gantry_arm_preset = { -0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        at_bins1234_.gantry_full_preset.insert(at_bins1234_.gantry_full_preset.begin(), at_bins1234_.gantry_torso_preset.begin(), at_bins1234_.gantry_torso_preset.end());
        at_bins1234_.gantry_full_preset.insert(at_bins1234_.gantry_full_preset.end(), at_bins1234_.gantry_arm_preset.begin(), at_bins1234_.gantry_arm_preset.end());
        
        //at bins 5, 6, 7, 8
        at_bins5678_.gantry_torso_preset = { -1.72, 3.0, -1.57 };
        at_bins5678_.gantry_arm_preset = { -0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        at_bins5678_.gantry_full_preset.insert(at_bins5678_.gantry_full_preset.begin(), at_bins5678_.gantry_torso_preset.begin(), at_bins5678_.gantry_torso_preset.end());
        at_bins5678_.gantry_full_preset.insert(at_bins5678_.gantry_full_preset.end(), at_bins5678_.gantry_arm_preset.begin(), at_bins5678_.gantry_arm_preset.end());
        

        // above bin1
        at_bin1_.gantry_torso_preset = { 0.06, -2.63, 0.0 };
        at_bin1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin1_.gantry_full_preset.insert(at_bin1_.gantry_full_preset.begin(), at_bin1_.gantry_torso_preset.begin(), at_bin1_.gantry_torso_preset.end());
        at_bin1_.gantry_full_preset.insert(at_bin1_.gantry_full_preset.end(), at_bin1_.gantry_arm_preset.begin(), at_bin1_.gantry_arm_preset.end());

        // above bin2
        at_bin2_.gantry_torso_preset = { 0.0, -3.35, -3.14 };
        at_bin2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin2_.gantry_full_preset.insert(at_bin2_.gantry_full_preset.begin(), at_bin2_.gantry_torso_preset.begin(), at_bin2_.gantry_torso_preset.end());
        at_bin2_.gantry_full_preset.insert(at_bin2_.gantry_full_preset.end(), at_bin2_.gantry_arm_preset.begin(), at_bin2_.gantry_arm_preset.end());

        // above bin3
        at_bin3_.gantry_torso_preset = { -0.78, -3.26, -3.14 };
        at_bin3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin3_.gantry_full_preset.insert(at_bin3_.gantry_full_preset.begin(), at_bin3_.gantry_torso_preset.begin(), at_bin3_.gantry_torso_preset.end());
        at_bin3_.gantry_full_preset.insert(at_bin3_.gantry_full_preset.end(), at_bin3_.gantry_arm_preset.begin(), at_bin3_.gantry_arm_preset.end());

        // above bin4
        at_bin4_.gantry_torso_preset = { -0.63, -2.63, 0.0 };
        at_bin4_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin4_.gantry_full_preset.insert(at_bin4_.gantry_full_preset.begin(), at_bin4_.gantry_torso_preset.begin(), at_bin4_.gantry_torso_preset.end());
        at_bin4_.gantry_full_preset.insert(at_bin4_.gantry_full_preset.end(), at_bin4_.gantry_arm_preset.begin(), at_bin4_.gantry_arm_preset.end());

        // above bin5
        at_bin5_.gantry_torso_preset = { 0.0, 2.63, 3.14 };
        at_bin5_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin5_.gantry_full_preset.insert(at_bin5_.gantry_full_preset.begin(), at_bin5_.gantry_torso_preset.begin(), at_bin5_.gantry_torso_preset.end());
        at_bin5_.gantry_full_preset.insert(at_bin5_.gantry_full_preset.end(), at_bin5_.gantry_arm_preset.begin(), at_bin5_.gantry_arm_preset.end());

        // above bin6
        at_bin6_.gantry_torso_preset = { 0.06, 3.36, 0.0 };
        at_bin6_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin6_.gantry_full_preset.insert(at_bin6_.gantry_full_preset.begin(), at_bin6_.gantry_torso_preset.begin(), at_bin6_.gantry_torso_preset.end());
        at_bin6_.gantry_full_preset.insert(at_bin6_.gantry_full_preset.end(), at_bin6_.gantry_arm_preset.begin(), at_bin6_.gantry_arm_preset.end());

        // above bin7
        at_bin7_.gantry_torso_preset = { -0.63, 3.36, 0.0 };
        at_bin7_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin7_.gantry_full_preset.insert(at_bin7_.gantry_full_preset.begin(), at_bin7_.gantry_torso_preset.begin(), at_bin7_.gantry_torso_preset.end());
        at_bin7_.gantry_full_preset.insert(at_bin7_.gantry_full_preset.end(), at_bin7_.gantry_arm_preset.begin(), at_bin7_.gantry_arm_preset.end());

        // above bin8
        at_bin8_.gantry_torso_preset = { -0.78, 2.72, 3.14 };
        at_bin8_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin8_.gantry_full_preset.insert(at_bin8_.gantry_full_preset.begin(), at_bin8_.gantry_torso_preset.begin(), at_bin8_.gantry_torso_preset.end());
        at_bin8_.gantry_full_preset.insert(at_bin8_.gantry_full_preset.end(), at_bin8_.gantry_arm_preset.begin(), at_bin8_.gantry_arm_preset.end());



        // above agv3_as3
        at_agv3_as3_.gantry_torso_preset = { -2.70, 1.40, 1.19 };
        at_agv3_as3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv3_as3_.gantry_full_preset.insert(at_agv3_as3_.gantry_full_preset.begin(), at_agv3_as3_.gantry_torso_preset.begin(), at_agv3_as3_.gantry_torso_preset.end());
        at_agv3_as3_.gantry_full_preset.insert(at_agv3_as3_.gantry_full_preset.end(), at_agv3_as3_.gantry_arm_preset.begin(), at_agv3_as3_.gantry_arm_preset.end());

        // above agv4_as3
        at_agv4_as3_.gantry_torso_preset = { -3.70, 3.99, 3.14 };
        at_agv4_as3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv4_as3_.gantry_full_preset.insert(at_agv4_as3_.gantry_full_preset.begin(), at_agv4_as3_.gantry_torso_preset.begin(), at_agv4_as3_.gantry_torso_preset.end());
        at_agv4_as3_.gantry_full_preset.insert(at_agv4_as3_.gantry_full_preset.end(), at_agv4_as3_.gantry_arm_preset.begin(), at_agv4_as3_.gantry_arm_preset.end());

        // above near_as3
        near_as3_.gantry_torso_preset = { -2.85, 2.68, 3.14 };
        near_as3_.gantry_arm_preset = { 0 , -1.65 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as3_.gantry_full_preset.insert(near_as3_.gantry_full_preset.begin(), near_as3_.gantry_torso_preset.begin(), near_as3_.gantry_torso_preset.end());
        near_as3_.gantry_full_preset.insert(near_as3_.gantry_full_preset.end(), near_as3_.gantry_arm_preset.begin(), near_as3_.gantry_arm_preset.end());

        // above as3
        at_as3_.gantry_torso_preset = { -3.87, 2.82, 1.44 };
        at_as3_.gantry_arm_preset = { 0 , -1.88 , 1.50 ,0.38 ,1.55 ,0.83 };
        at_as3_.gantry_full_preset.insert(at_as3_.gantry_full_preset.begin(), at_as3_.gantry_torso_preset.begin(), at_as3_.gantry_torso_preset.end());
        at_as3_.gantry_full_preset.insert(at_as3_.gantry_full_preset.end(), at_as3_.gantry_arm_preset.begin(), at_as3_.gantry_arm_preset.end());


        // above agv1_as1
        at_agv1_as1_.gantry_torso_preset = { -2.70, -4.5, 1.19 };
        at_agv1_as1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv1_as1_.gantry_full_preset.insert(at_agv1_as1_.gantry_full_preset.begin(), at_agv1_as1_.gantry_torso_preset.begin(), at_agv1_as1_.gantry_torso_preset.end());
        at_agv1_as1_.gantry_full_preset.insert(at_agv1_as1_.gantry_full_preset.end(), at_agv1_as1_.gantry_arm_preset.begin(), at_agv1_as1_.gantry_arm_preset.end());

        // above agv2_as1
        at_agv2_as1_.gantry_torso_preset = { -3.7, -2.13, 3.14 };
        at_agv2_as1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv2_as1_.gantry_full_preset.insert(at_agv2_as1_.gantry_full_preset.begin(), at_agv2_as1_.gantry_torso_preset.begin(), at_agv2_as1_.gantry_torso_preset.end());
        at_agv2_as1_.gantry_full_preset.insert(at_agv2_as1_.gantry_full_preset.end(), at_agv2_as1_.gantry_arm_preset.begin(), at_agv2_as1_.gantry_arm_preset.end());

        // above near_as1
        near_as1_.gantry_torso_preset = { -2.5, -3.21, 3.14 };
        near_as1_.gantry_arm_preset = { 0 , -1.65 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as1_.gantry_full_preset.insert(near_as1_.gantry_full_preset.begin(), near_as1_.gantry_torso_preset.begin(), near_as1_.gantry_torso_preset.end());
        near_as1_.gantry_full_preset.insert(near_as1_.gantry_full_preset.end(), near_as1_.gantry_arm_preset.begin(), near_as1_.gantry_arm_preset.end());

        // above as1
        at_as1_.gantry_torso_preset = { -3.87, -3.07, 1.44 };
        at_as1_.gantry_arm_preset = { 0 , -1.88 , 1.50 ,0.38 ,1.55 ,0.83 };
        at_as1_.gantry_full_preset.insert(at_as1_.gantry_full_preset.begin(), at_as1_.gantry_torso_preset.begin(), at_as1_.gantry_torso_preset.end());
        at_as1_.gantry_full_preset.insert(at_as1_.gantry_full_preset.end(), at_as1_.gantry_arm_preset.begin(), at_as1_.gantry_arm_preset.end());


        // above agv1_as2
        at_agv1_as2_.gantry_torso_preset = { -7.75, -4.5, 1.19 };
        at_agv1_as2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv1_as2_.gantry_full_preset.insert(at_agv1_as2_.gantry_full_preset.begin(), at_agv1_as2_.gantry_torso_preset.begin(), at_agv1_as2_.gantry_torso_preset.end());
        at_agv1_as2_.gantry_full_preset.insert(at_agv1_as2_.gantry_full_preset.end(), at_agv1_as2_.gantry_arm_preset.begin(), at_agv1_as2_.gantry_arm_preset.end());

        // above agv2_as2
        at_agv2_as2_.gantry_torso_preset = { -8.7, -2.13, 3.14 };
        at_agv2_as2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv2_as2_.gantry_full_preset.insert(at_agv2_as2_.gantry_full_preset.begin(), at_agv2_as2_.gantry_torso_preset.begin(), at_agv2_as2_.gantry_torso_preset.end());
        at_agv2_as2_.gantry_full_preset.insert(at_agv2_as2_.gantry_full_preset.end(), at_agv2_as2_.gantry_arm_preset.begin(), at_agv2_as2_.gantry_arm_preset.end());

        // above near_as2
        near_as2_.gantry_torso_preset = { -7.5, -3.21, 3.14 };
        near_as2_.gantry_arm_preset = { 0 , -1.65 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as2_.gantry_full_preset.insert(near_as2_.gantry_full_preset.begin(), near_as2_.gantry_torso_preset.begin(), near_as2_.gantry_torso_preset.end());
        near_as2_.gantry_full_preset.insert(near_as2_.gantry_full_preset.end(), near_as2_.gantry_arm_preset.begin(), near_as2_.gantry_arm_preset.end());

        // above as2
        at_as2_.gantry_torso_preset = { -8.87, -3.07, 1.44 };
        at_as2_.gantry_arm_preset = { 0 , -1.88 , 1.50 ,0.38 ,1.55 ,0.83 };
        at_as2_.gantry_full_preset.insert(at_as2_.gantry_full_preset.begin(), at_as2_.gantry_torso_preset.begin(), at_as2_.gantry_torso_preset.end());
        at_as2_.gantry_full_preset.insert(at_as2_.gantry_full_preset.end(), at_as2_.gantry_arm_preset.begin(), at_as2_.gantry_arm_preset.end());

        // above agv3_as4
        at_agv3_as4_.gantry_torso_preset = {  -7.75, 1.40, 1.19 };
        at_agv3_as4_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv3_as4_.gantry_full_preset.insert(at_agv3_as4_.gantry_full_preset.begin(), at_agv3_as4_.gantry_torso_preset.begin(), at_agv3_as4_.gantry_torso_preset.end());
        at_agv3_as4_.gantry_full_preset.insert(at_agv3_as4_.gantry_full_preset.end(), at_agv3_as4_.gantry_arm_preset.begin(), at_agv3_as4_.gantry_arm_preset.end());

        // above agv4_as4
        at_agv4_as4_.gantry_torso_preset = { -8.70, 3.99, 3.14 };
        at_agv4_as4_.gantry_arm_preset = {  0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv4_as4_.gantry_full_preset.insert(at_agv4_as4_.gantry_full_preset.begin(), at_agv4_as4_.gantry_torso_preset.begin(), at_agv4_as4_.gantry_torso_preset.end());
        at_agv4_as4_.gantry_full_preset.insert(at_agv4_as4_.gantry_full_preset.end(), at_agv4_as4_.gantry_arm_preset.begin(), at_agv4_as4_.gantry_arm_preset.end());

        // above near_as4
        near_as4_.gantry_torso_preset = { -7.85, 2.68, 3.14 };
        near_as4_.gantry_arm_preset = {  0 , -1.65 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as4_.gantry_full_preset.insert(near_as4_.gantry_full_preset.begin(), near_as4_.gantry_torso_preset.begin(), near_as4_.gantry_torso_preset.end());
        near_as4_.gantry_full_preset.insert(near_as4_.gantry_full_preset.end(), near_as4_.gantry_arm_preset.begin(), near_as4_.gantry_arm_preset.end());

        // above as4
        at_as4_.gantry_torso_preset = { -8.87, 2.82, 1.44 };
        at_as4_.gantry_arm_preset = { 0 , -1.88 , 1.50 ,0.38 ,1.55 ,0.83 };
        at_as4_.gantry_full_preset.insert(at_as4_.gantry_full_preset.begin(), at_as4_.gantry_torso_preset.begin(), at_as4_.gantry_torso_preset.end());
        at_as4_.gantry_full_preset.insert(at_as4_.gantry_full_preset.end(), at_as4_.gantry_arm_preset.begin(), at_as4_.gantry_arm_preset.end());


        // above agv1
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv1_.gantry_torso_preset = { -0.37, -3.78, -0.69 };
        at_agv1_.gantry_arm_preset = { -0.01, -1.17, 1.20, -0.01, 1.54, 0.83 };
        at_agv1_.gantry_full_preset.insert(at_agv1_.gantry_full_preset.begin(), at_agv1_.gantry_torso_preset.begin(), at_agv1_.gantry_torso_preset.end());
        at_agv1_.gantry_full_preset.insert(at_agv1_.gantry_full_preset.end(), at_agv1_.gantry_arm_preset.begin(), at_agv1_.gantry_arm_preset.end());

        // above agv2
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv2_.gantry_torso_preset = { -0.22, -2.16, -3.14 };
        at_agv2_.gantry_arm_preset = { -0.01, -1.17, 1.20, -0.01, 1.54, 0.83 };
        at_agv2_.gantry_full_preset.insert(at_agv2_.gantry_full_preset.begin(), at_agv2_.gantry_torso_preset.begin(), at_agv2_.gantry_torso_preset.end());
        at_agv2_.gantry_full_preset.insert(at_agv2_.gantry_full_preset.end(), at_agv2_.gantry_arm_preset.begin(), at_agv2_.gantry_arm_preset.end());

        // above agv3
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv3_.gantry_torso_preset = { -0.37, 2.10, -0.69 };
        at_agv3_.gantry_arm_preset = { -0.01, -1.17, 1.20, -0.01, 1.54, 0.83 };
        at_agv3_.gantry_full_preset.insert(at_agv3_.gantry_full_preset.begin(), at_agv3_.gantry_torso_preset.begin(), at_agv3_.gantry_torso_preset.end());
        at_agv3_.gantry_full_preset.insert(at_agv3_.gantry_full_preset.end(), at_agv3_.gantry_arm_preset.begin(), at_agv3_.gantry_arm_preset.end());

        // above agv4
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv4_.gantry_torso_preset = { -0.22, 3.67, -3.14 };
        at_agv4_.gantry_arm_preset = { -0.01, -1.17, 1.20, -0.01, 1.54, 0.83 };
        at_agv4_.gantry_full_preset.insert(at_agv4_.gantry_full_preset.begin(), at_agv4_.gantry_torso_preset.begin(), at_agv4_.gantry_torso_preset.end());
        at_agv4_.gantry_full_preset.insert(at_agv4_.gantry_full_preset.end(), at_agv4_.gantry_arm_preset.begin(), at_agv4_.gantry_arm_preset.end());


        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
        const moveit::core::JointModelGroup* joint_model_group =
            full_gantry_group_.getCurrentState()->getJointModelGroup("gantry_full");
        moveit::core::RobotStatePtr current_state = full_gantry_group_.getCurrentState();
        // next get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);


        const moveit::core::JointModelGroup* joint_arm_group =
            arm_gantry_group_.getCurrentState()->getJointModelGroup("gantry_arm");
        moveit::core::RobotStatePtr current_state_arm = arm_gantry_group_.getCurrentState();
        current_state_arm->copyJointGroupPositions(joint_arm_group, joint_arm_positions_);
    }


    /**
     * @brief Pick up a part from a bin
     *
     * @param part Part to pick up
     * @return true Part was picked up
     * @return false Part was not picked up
     *
     * We use the group full_gantry_group_ to allow the robot more flexibility
     */
    bool Gantry::pickPart(geometry_msgs::Pose part_init_pose_in_world)
    {
        ros::Duration(0.5).sleep();
        activateGripper();
        const double GRIPPER_HEIGHT = 0.01;
        const double EPSILON = 0.008; // for the gripper to firmly touch
        ros::Duration(0.5).sleep();

        // pose of the end effector in the world frame
        geometry_msgs::Pose gantry_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;


        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        
        gantry_ee_link_pose.orientation.x = flat_orientation.getX();
        gantry_ee_link_pose.orientation.y = flat_orientation.getY();
        gantry_ee_link_pose.orientation.z = flat_orientation.getZ();
        gantry_ee_link_pose.orientation.w = flat_orientation.getW();

        tf2::Quaternion  gantry_ee_link_orientation(
            gantry_ee_link_pose.orientation.x,
            gantry_ee_link_pose.orientation.y,
            gantry_ee_link_pose.orientation.z,
            gantry_ee_link_pose.orientation.w
        );

     

        // pose to go to after grasping a part (lift the arm a little bit)
        geometry_msgs::Pose postGraspPose;

        part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + 0.08;
        part_init_pose_in_world.orientation.x = gantry_ee_link_pose.orientation.x;
        part_init_pose_in_world.orientation.y = gantry_ee_link_pose.orientation.y;
        part_init_pose_in_world.orientation.z = gantry_ee_link_pose.orientation.z;
        part_init_pose_in_world.orientation.w = gantry_ee_link_pose.orientation.w;

        // activate gripper
        ros::Duration(3).sleep();
        activateGripper();
        auto state = getGripperState();


        while (!state.enabled) {
            activateGripper();
            state = getGripperState();
        }

        if (!state.enabled) {
            ROS_FATAL_STREAM("[Gripper] = Could not enable gripper...shutting down");
            ros::shutdown();
        }

        if (state.enabled) {
            ROS_INFO_STREAM("[Gripper] = enabled");
            //--Move arm to part
            arm_gantry_group_.setPoseTarget(part_init_pose_in_world);
            arm_gantry_group_.move();

            state = getGripperState();
            // move the arm closer until the object is attached
            while (!state.attached) {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                arm_gantry_group_.setPoseTarget(part_init_pose_in_world);
                arm_gantry_group_.move();
                arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
                state = getGripperState();
            }

            ROS_INFO_STREAM("[Gripper] = object attached");
            // ros::Duration(1.0).sleep();
            postGraspPose = arm_gantry_group_.getCurrentPose().pose;
            postGraspPose.position.z = postGraspPose.position.z + 0.2;
            //--Move arm to previous position
            arm_gantry_group_.setPoseTarget(postGraspPose);
            arm_gantry_group_.move();
            ros::Duration(2.0).sleep();
            arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
            arm_gantry_group_.move();
            return true;
        }
        return false;
    }


    /////////////////////////////////////////////////////
    bool Gantry::placePart(geometry_msgs::Pose part_init_pose_in_world, geometry_msgs::Pose target_pose_in_frame, std::string location)
    {

        ROS_INFO_STREAM("arm: " << arm_gantry_group_.getCurrentPose().pose.orientation);
        ROS_INFO_STREAM("full: " << full_gantry_group_.getCurrentPose().pose.orientation);

        // get the target pose of the part in the world frame
        auto target_in_world_frame = motioncontrol::transformtoWorldFrame(
            target_pose_in_frame,
            location);

        if (location == "agv1") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_agv1_);
        }
        if (location == "agv2") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_agv2_);
        }
        if (location == "agv3") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_agv3_);
        }
        if (location == "agv4") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_agv4_);
        }
        if (location == "as1") {
            goToPresetLocation(at_as1_);
        }
        if (location == "as2") {
            goToPresetLocation(at_as2_);
        }
        if (location == "as3") {
            goToPresetLocation(at_as3_);
        }
        if (location == "as4") {
            goToPresetLocation(at_as4_);
        }

        ROS_INFO("Target World Position: %f, %f, %f",
            target_in_world_frame.position.x,
            target_in_world_frame.position.y,
            target_in_world_frame.position.z);

        ROS_INFO("Target World Orientation: %f, %f, %f, %f",
            target_in_world_frame.orientation.x,
            target_in_world_frame.orientation.y,
            target_in_world_frame.orientation.z,
            target_in_world_frame.orientation.w);

        auto ee_pose = arm_gantry_group_.getCurrentPose().pose;

        tf2::Quaternion q_current(
            ee_pose.orientation.x,
            ee_pose.orientation.y,
            ee_pose.orientation.z,
            ee_pose.orientation.w);

        // orientation of the part in the bin, in world frame
        tf2::Quaternion q_init_part(
            part_init_pose_in_world.orientation.x,
            part_init_pose_in_world.orientation.y,
            part_init_pose_in_world.orientation.z,
            part_init_pose_in_world.orientation.w);
        // orientation of the part in the tray, in world frame
        tf2::Quaternion q_target_part(
            target_in_world_frame.orientation.x,
            target_in_world_frame.orientation.y,
            target_in_world_frame.orientation.z,
            target_in_world_frame.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        // orientation of the gripper when placing the part in the tray
        target_in_world_frame.orientation.x = q_rslt.x();
        target_in_world_frame.orientation.y = q_rslt.y();
        target_in_world_frame.orientation.z = q_rslt.z();
        target_in_world_frame.orientation.w = q_rslt.w();
        target_in_world_frame.position.z += 0.17;

        //allow replanning if it fails

        arm_gantry_group_.setPoseTarget(target_in_world_frame);
        arm_gantry_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        auto state = getGripperState();
        if (state.attached)
            return true;
        else
            return false;
        // TODO: check the part was actually placed in the correct pose in the agv
        // and that it is not faulty
    }


    bool Gantry::movePart(geometry_msgs::Pose part_init_pose_in_world, geometry_msgs::Pose target_pose_in_frame, std::string location, std::string type){

        ROS_INFO_STREAM("in gantry movePart");
           
        // orientation of the part in the bin, in world frame
        tf2::Quaternion q_init_part(
            part_init_pose_in_world.orientation.x,
            part_init_pose_in_world.orientation.y,
            part_init_pose_in_world.orientation.z,
            part_init_pose_in_world.orientation.w);

        // get the target pose of the part in the world frame
        auto target_in_world_frame = motioncontrol::transformtoWorldFrame(
            target_pose_in_frame,
            location);

        // orientation of the part in the tray, in world frame
        tf2::Quaternion q_target_part(
            target_in_world_frame.orientation.x,
            target_in_world_frame.orientation.y,
            target_in_world_frame.orientation.z,
            target_in_world_frame.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        
        // pose of the end effector in the world frame
        geometry_msgs::Pose gantry_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        
        gantry_ee_link_pose.orientation.x = flat_orientation.getX();
        gantry_ee_link_pose.orientation.y = flat_orientation.getY();
        gantry_ee_link_pose.orientation.z = flat_orientation.getZ();
        gantry_ee_link_pose.orientation.w = flat_orientation.getW();
        
        double z_add {0.07};

        if (type.find("pump") != std::string::npos){
            z_add = 0.08;
        }
        if (type.find("battery") != std::string::npos){
            z_add = 0.06;
        }
        
    
        part_init_pose_in_world.orientation = gantry_ee_link_pose.orientation;
        part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + z_add;
        // pose to go to after grasping a part (lift the arm a little bit)
        geometry_msgs::Pose postGraspPose;
        

        // activate gripper
        ros::Duration(0.5).sleep();
        activateGripper();
        auto state = getGripperState();


        while (!state.enabled) {
            activateGripper();
            state = getGripperState();
        }

        if (!state.enabled) {
            ROS_FATAL_STREAM("[Gripper] = Could not enable gripper...shutting down");
            ros::shutdown();
        }

        if (state.enabled) {
            ROS_INFO_STREAM("[Gripper] = enabled");
            //--Move arm to part
            arm_gantry_group_.setPoseTarget(part_init_pose_in_world);
            arm_gantry_group_.move();

            state = getGripperState();
            // move the arm closer until the object is attached
            while (!state.attached) {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                arm_gantry_group_.setPoseTarget(part_init_pose_in_world);
                arm_gantry_group_.move();
                arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
                state = getGripperState();
            }

            ROS_INFO_STREAM("[Gripper] = object attached");
            // ros::Duration(1.0).sleep();
            postGraspPose = arm_gantry_group_.getCurrentPose().pose;
            postGraspPose.position.z = postGraspPose.position.z + 0.2;
            //--Move arm to previous position
            arm_gantry_group_.setPoseTarget(postGraspPose);
            arm_gantry_group_.move();
            ros::Duration(2.0).sleep();
            arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
            arm_gantry_group_.move();
        }

        double z_t{0.0};

        if (location == "agv1") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_agv1_);
            z_t = 0.18;
        }
        if (location == "agv2") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_agv2_);
            z_t = 0.18;

        }
        if (location == "agv3") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_agv3_);
            z_t = 0.18;

        }
        if (location == "agv4") {
            goToPresetLocation(home_);
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_agv4_);
            z_t = 0.18;

        }
        if (location == "as1") {
            goToPresetLocation(near_as1_);
            goToPresetLocation(at_as1_);
            z_t = 0.05;

        }
        if (location == "as2") {
            goToPresetLocation(near_as2_);
            goToPresetLocation(at_as2_);
            z_t = 0.05;

        }
        if (location == "as3") {
            goToPresetLocation(near_as3_);
            goToPresetLocation(at_as3_);
            z_t = 0.05;

        }
        if (location == "as4") {
            goToPresetLocation(near_as4_);
            goToPresetLocation(at_as4_);
            z_t = 0.05;

        }


        tf2::Quaternion q_current(
            gantry_ee_link_pose.orientation.x,
            gantry_ee_link_pose.orientation.y,
            gantry_ee_link_pose.orientation.z,
            gantry_ee_link_pose.orientation.w);
         
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        geometry_msgs::Pose arm_pose = arm_gantry_group_.getCurrentPose().pose;

        // orientation of the gripper when placing the part in the tray
        arm_pose.orientation.x = q_rslt.x();
        arm_pose.orientation.y = q_rslt.y();
        arm_pose.orientation.z = q_rslt.z();
        arm_pose.orientation.w = q_rslt.w();
        arm_pose.position.x = target_in_world_frame.position.x;
        arm_pose.position.y = target_in_world_frame.position.y;
        arm_pose.position.z = target_in_world_frame.position.z + z_t;


        arm_gantry_group_.setPoseTarget(arm_pose);
        arm_gantry_group_.move();

        ros::Duration(2.0).sleep();
        deactivateGripper();

        if (location == "as1") {
            goToPresetLocation(at_as1_);
            goToPresetLocation(near_as1_);
            
        }
        if (location == "as2") {
            goToPresetLocation(at_as2_);
            goToPresetLocation(near_as2_);

        }
        if (location == "as3") {
            goToPresetLocation(at_as3_);
            goToPresetLocation(near_as3_);

        }
        if (location == "as4") {
            goToPresetLocation(at_as4_);
            goToPresetLocation(near_as4_);

        }

        if (location == "agv1") {
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(home_);

        }
        if (location == "agv2") {
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(home_);
        }
        if (location == "agv3") {
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(home_);
        }
        if (location == "agv4") {
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(home_);
        }



        auto state1 = getGripperState();
        if (state1.attached)
            return true;
        else
            return false;

    }


    bool Gantry::movePartfrombin(geometry_msgs::Pose part_init_pose_in_world, std::string type, unsigned short int bin){


        geometry_msgs::Pose target_in_world_frame;
        std::array<double,3> bin_origin{0,0,0};
        auto place_orientation = motioncontrol::quaternionFromEuler(0, 0, -1.57);
        // bin = 5;
        if (bin == 1){
            bin_origin = bin1_origin_;
        }
        if (bin == 2){
            bin_origin = bin2_origin_;
        }
        if (bin == 3){
            bin_origin = bin3_origin_;
        }
        if (bin == 4){
            bin_origin = bin4_origin_;
        }
        if (bin == 5){
            bin_origin = bin5_origin_;
        }
        if (bin == 6){
            bin_origin = bin6_origin_;
        }
        if (bin == 7){
            bin_origin = bin7_origin_;
        }
        if (bin == 8){
            bin_origin = bin8_origin_;
        }
        target_in_world_frame.position.x = bin_origin.at(0);
        target_in_world_frame.position.y = bin_origin.at(1) - 0.2;
        target_in_world_frame.position.z = bin_origin.at(2) + 0.05;
        target_in_world_frame.orientation.x = place_orientation.getX();
        target_in_world_frame.orientation.y = place_orientation.getY();
        target_in_world_frame.orientation.z = place_orientation.getZ();
        target_in_world_frame.orientation.w = place_orientation.getW();


        // orientation of the part in the bin, in world frame
        tf2::Quaternion q_init_part(
            part_init_pose_in_world.orientation.x,
            part_init_pose_in_world.orientation.y,
            part_init_pose_in_world.orientation.z,
            part_init_pose_in_world.orientation.w);


        // orientation of the part in the tray, in world frame
        tf2::Quaternion q_target_part(
            target_in_world_frame.orientation.x,
            target_in_world_frame.orientation.y,
            target_in_world_frame.orientation.z,
            target_in_world_frame.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        
        // pose of the end effector in the world frame
        geometry_msgs::Pose gantry_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        
        gantry_ee_link_pose.orientation.x = flat_orientation.getX();
        gantry_ee_link_pose.orientation.y = flat_orientation.getY();
        gantry_ee_link_pose.orientation.z = flat_orientation.getZ();
        gantry_ee_link_pose.orientation.w = flat_orientation.getW();
        
        double z_add {0.07};

        if (type.find("pump") != std::string::npos){
            z_add = 0.08;
        }
        if (type.find("battery") != std::string::npos){
            z_add = 0.06;
        }
    
        part_init_pose_in_world.orientation = gantry_ee_link_pose.orientation;
        part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + z_add;

        geometry_msgs::Pose postGraspPose;

        // activate gripper
        ros::Duration(0.5).sleep();
        activateGripper();
        auto state = getGripperState();


        while (!state.enabled) {
            activateGripper();
            state = getGripperState();
        }

        if (!state.enabled) {
            ROS_FATAL_STREAM("[Gripper] = Could not enable gripper...shutting down");
            ros::shutdown();
        }

        if (state.enabled) {
            ROS_INFO_STREAM("[Gripper] = enabled");
            //--Move arm to part
            arm_gantry_group_.setPoseTarget(part_init_pose_in_world);
            arm_gantry_group_.move();

            state = getGripperState();
            // move the arm closer until the object is attached
            while (!state.attached) {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                arm_gantry_group_.setPoseTarget(part_init_pose_in_world);
                arm_gantry_group_.move();
                arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
                state = getGripperState();
            }

            ROS_INFO_STREAM("[Gripper] = object attached");
            // gantry_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
            // ros::Duration(1.0).sleep();
            postGraspPose = arm_gantry_group_.getCurrentPose().pose;
            postGraspPose.position.z = postGraspPose.position.z + 0.1;
            //--Move arm to previous position
            arm_gantry_group_.setPoseTarget(postGraspPose);
            arm_gantry_group_.move();
            ros::Duration(2.0).sleep();
            arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
            arm_gantry_group_.move();
        }

        goToPresetLocation(home_);
        gantry_ee_link_pose.orientation.x = flat_orientation.getX();
        gantry_ee_link_pose.orientation.y = flat_orientation.getY();
        gantry_ee_link_pose.orientation.z = flat_orientation.getZ();
        gantry_ee_link_pose.orientation.w = flat_orientation.getW();
        arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
        arm_gantry_group_.move();

        if (bin == 1) {
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_bin1_);
        }
        if (bin == 2) {
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_bin2_);
        }
        if (bin == 5) {
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_bin5_);
        }
        if (bin == 6) {
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_bin6_);
        }
   

        tf2::Quaternion q_current(
            gantry_ee_link_pose.orientation.x,
            gantry_ee_link_pose.orientation.y,
            gantry_ee_link_pose.orientation.z,
            gantry_ee_link_pose.orientation.w);
         
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        geometry_msgs::Pose arm_pose = arm_gantry_group_.getCurrentPose().pose;
        // orientation of the gripper when placing the part in the tray
        arm_pose.orientation.x = q_rslt.x();
        arm_pose.orientation.y = q_rslt.y();
        arm_pose.orientation.z = q_rslt.z();
        arm_pose.orientation.w = q_rslt.w();
        arm_gantry_group_.setPoseTarget(arm_pose);
        arm_gantry_group_.move();
        

        arm_pose.position.z = target_in_world_frame.position.z + 0.2;
        arm_pose.position.x = target_in_world_frame.position.x;
        arm_pose.position.y = target_in_world_frame.position.y-0.05;

        arm_gantry_group_.setPoseTarget(arm_pose);
        arm_gantry_group_.move();

        ros::Duration(2.0).sleep();
        deactivateGripper();
        goToPresetLocation(home_);

        auto state1 = getGripperState();
        if (state1.attached)
            return true;
        else
            return false;

    }

    void Gantry::flippart(Product part, std::vector<int> empty_bins, geometry_msgs::Pose part_pose_in_frame, std::string agv, bool arm_required){
        std::string part_type = part.type;
        geometry_msgs::Pose part_pose = part.world_pose;
        geometry_msgs::Pose ppf = part_pose_in_frame;
        ROS_INFO_STREAM("In flip");
        
        int bin_selected = 0;
        for(auto &bin: empty_bins){
            ROS_INFO_STREAM("bin number "<< bin);
            if(bin == 1 || bin == 2 || bin == 3 || bin == 4 || bin == 5 || bin == 6 || bin == 7 || bin == 8)
            {
                bin_selected = bin;
                break;
            }
        }
        // bin_selected = 5;
        // ROS_INFO_STREAM("selected bin number "<< bin_selected);
        std::array<double,3> bin_origin{0,0,0};
        geometry_msgs::Pose part_world_pose;
        if (bin_selected == 1){
            bin_origin = bin1_origin_;
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_bin1_);
        }
        if (bin_selected == 2){
            bin_origin = bin2_origin_;
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_bin2_);

        }
        if (bin_selected == 3){
            bin_origin = bin3_origin_;
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_bin3_);

        }
        if (bin_selected == 4){
            bin_origin = bin4_origin_;
            goToPresetLocation(at_bins1234_);
            goToPresetLocation(at_bin4_);

        }
        if (bin_selected == 5){
            bin_origin = bin5_origin_;
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_bin5_);

        }
        if (bin_selected == 6){
            bin_origin = bin6_origin_;
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_bin6_);

        }
        if (bin_selected == 7){
            bin_origin = bin7_origin_;
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_bin7_);

        }
        if (bin_selected == 8){
            bin_origin = bin8_origin_;
            goToPresetLocation(at_bins5678_);
            goToPresetLocation(at_bin8_);

        }
        // ROS_INFO_STREAM("EMPTYBIN: "<<bin_selected);

        part.bin_number = bin_selected;
               
       

        geometry_msgs::Pose arm_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();
        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1);
        arm_ee_link_pose.position.z = bin_origin.at(2)+0.1;

        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();
        
        tf2::Quaternion q_current(
            arm_ee_link_pose.orientation.x,
            arm_ee_link_pose.orientation.y,
            arm_ee_link_pose.orientation.z,
            arm_ee_link_pose.orientation.w);
        auto target_pose = motioncontrol::quaternionFromEuler(0, 0, -1.57);
        tf2::Quaternion q_init_part(
            part_pose.orientation.x,
            part_pose.orientation.y,
            part_pose.orientation.z,
            part_pose.orientation.w);
        tf2::Quaternion q_target_part(
            target_pose.getX(),
            target_pose.getY(),
            target_pose.getZ(),
            target_pose.getW());
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();
        // orientation of the gripper when placing the part in the tray
        arm_ee_link_pose.orientation.x = q_rslt.x();
        arm_ee_link_pose.orientation.y = q_rslt.y();
        arm_ee_link_pose.orientation.z = q_rslt.z();
        arm_ee_link_pose.orientation.w = q_rslt.w();
      
        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        ros::Duration(2.0).sleep();
        // moveBaseTo(bin_origin.at(1)-0.4);
        part_pose.position.x = arm_ee_link_pose.position.x;
        part_pose.position.y = arm_ee_link_pose.position.y;
        part_pose.position.z = 0.8;
        part_pose.orientation.x = target_pose.getX();
        part_pose.orientation.y = target_pose.getY();
        part_pose.orientation.z = target_pose.getZ();
        part_pose.orientation.w = target_pose.getW();

        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1);
        arm_ee_link_pose.position.z = bin_origin.at(2) + 0.3;
        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();

        arm_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        geometry_msgs::Pose Post_grasp = arm_ee_link_pose;
        
        auto side_orientation = motioncontrol::quaternionFromEuler(0, 0, -1.57);
        arm_ee_link_pose.orientation.x = side_orientation.getX();
        arm_ee_link_pose.orientation.y = side_orientation.getY();
        arm_ee_link_pose.orientation.z = side_orientation.getZ();
        arm_ee_link_pose.orientation.w = side_orientation.getW();
        // target_pose.position.z = bin_origin.at(2)+0.2;
        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();
        arm_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        geometry_msgs::Pose Post_grasp1 = arm_ee_link_pose;
        
        // activate gripper
        ros::Duration(0.5).sleep();
        activateGripper();
        auto state = getGripperState();

        while (!state.enabled) {
            activateGripper();
            state = getGripperState();
        }
        
        arm_ee_link_pose.position.x = part_pose.position.x;
        arm_ee_link_pose.position.y = part_pose.position.y + 0.12 ;
        arm_ee_link_pose.position.z = part_pose.position.z;
     
        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();

        state = getGripperState();
        while (!state.attached) {
            arm_ee_link_pose.position.y -= 0.005;
            arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
            arm_gantry_group_.move();
            state = getGripperState();
            ros::Duration(sleep(0.5));
        }

        arm_ee_link_pose.position.z = arm_ee_link_pose.position.z + 0.12;
      
        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();
        
        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1)+0.07;
       
        arm_gantry_group_.setPoseTarget(arm_ee_link_pose);
        arm_gantry_group_.move();
        
        const moveit::core::JointModelGroup* joint_model_group =
            arm_gantry_group_.getCurrentState()->getJointModelGroup("gantry_arm");
        moveit::core::RobotStatePtr current_state = arm_gantry_group_.getCurrentState();

        // get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

        // next, assign a value to only the linear_arm_actuator_joint
        
        ROS_INFO_STREAM("wrist3 " << joint_group_positions_.at(5));
        joint_group_positions_.at(5) = joint_group_positions_.at(5) + M_PI;
        // move the arm
        ROS_INFO_STREAM("wrist3 "<<joint_group_positions_.at(5));
        arm_gantry_group_.setJointValueTarget(joint_group_positions_);
        full_gantry_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        ROS_INFO_STREAM("dropped");
        joint_group_positions_.at(5) = joint_group_positions_.at(5) - M_PI;
        ROS_INFO_STREAM("wrist3 " << joint_group_positions_.at(5));
        arm_gantry_group_.setJointValueTarget(joint_group_positions_);
        full_gantry_group_.move();
        ros::Duration(2.0).sleep();
        part.world_pose.position.x = bin_origin.at(0);
        part.world_pose.position.y = bin_origin.at(1);
        part.world_pose.position.z = 0.8;
        ROS_INFO_STREAM(part.world_pose);
        auto final_orientation = motioncontrol::quaternionFromEuler(3.14, 0, -1.57);
        part.world_pose.orientation.x = final_orientation.getX();
        part.world_pose.orientation.y = final_orientation.getY();
        part.world_pose.orientation.z = final_orientation.getZ();
        part.world_pose.orientation.w = final_orientation.getW();
        ROS_INFO_STREAM(part.world_pose);
        try{
            
            flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
            arm_ee_link_pose.orientation.x = flat_orientation.getX();
            arm_ee_link_pose.orientation.y = flat_orientation.getY();
            arm_ee_link_pose.orientation.z = flat_orientation.getZ();
            arm_ee_link_pose.orientation.w = flat_orientation.getW();
            ROS_INFO_STREAM("going to post_grasp1");
            arm_gantry_group_.setPoseTarget(Post_grasp1);
            arm_gantry_group_.move();
            ROS_INFO_STREAM("Reached post_grasp1");
            ros::Duration(2.0).sleep();
            ROS_INFO_STREAM("going to post_grasp");
            arm_gantry_group_.setPoseTarget(Post_grasp);
            arm_gantry_group_.move();
            ROS_INFO_STREAM("Reached post_grasp");

            if (part.bin_number == 1){
                ROS_INFO_STREAM("in 1");
                goToPresetLocation(at_bin1_);
            }
            else if (part.bin_number == 2){
                ROS_INFO_STREAM("in 2");
                goToPresetLocation(at_bin2_);

            }
            else if (part.bin_number == 3){
                ROS_INFO_STREAM("in 3");
                goToPresetLocation(at_bin3_);

            }
            else if (part.bin_number== 4){
                ROS_INFO_STREAM("in 4");
                goToPresetLocation(at_bin4_);

            }
            else if (part.bin_number == 5){
                ROS_INFO_STREAM("in 5");
                goToPresetLocation(at_bin5_);

            }
            else if (part.bin_number == 6){
                ROS_INFO_STREAM("in 6");
                goToPresetLocation(at_bin6_);

            }
            else if (part.bin_number == 7){
                ROS_INFO_STREAM("in 7");
                goToPresetLocation(at_bin7_);

            }
            else if (part.bin_number == 8){
                ROS_INFO_STREAM("in 8");
                goToPresetLocation(at_bin8_);

            }
            
            ROS_INFO_STREAM(part.bin_number);
            goToPresetLocation(home_);
            ROS_INFO_STREAM("Home reached");
            ros::Duration(2.0).sleep();
            ROS_INFO_STREAM("going to pick part");
            move_gantry_to_bin(part.bin_number);
        }
        catch(std::string agv){
            ROS_INFO_STREAM(agv);
            movePart(part.world_pose, ppf, agv, part_type);
        }
    }
    
    void Gantry::move_gantry_to_bin(unsigned short int bin){
        if(bin == 1){
            goToPresetLocation(at_bin1_);
        }
        if(bin == 2){
            goToPresetLocation(at_bin2_);
        }
        if(bin == 3){
            goToPresetLocation(at_bin3_);
        }
        if(bin == 4){
            goToPresetLocation(at_bin4_);
        }
        if(bin == 5){
            goToPresetLocation(at_bin5_);
        }
        if(bin == 6){
            goToPresetLocation(at_bin6_);
        }
        if(bin == 7){
            goToPresetLocation(at_bin7_);
        }
        if(bin == 8){
            goToPresetLocation(at_bin8_);
        }
    }

    void Gantry::move_gantry_to_assembly_station(std::string c_name){
        if (c_name.find("as1") != std::string::npos){
            goToPresetLocation(near_as1_);
            if (c_name.find("agv1") != std::string::npos){
                goToPresetLocation(at_agv1_as1_);
            }
            if (c_name.find("agv2") != std::string::npos){
                goToPresetLocation(at_agv2_as1_);
            }
        }
        if (c_name.find("as2") != std::string::npos){
            goToPresetLocation(home2_);
            goToPresetLocation(near_as2_);
            if (c_name.find("agv1") != std::string::npos){
                goToPresetLocation(at_agv1_as2_);
            }
            if (c_name.find("agv2") != std::string::npos){
                goToPresetLocation(at_agv2_as2_);
            }
        }
        if (c_name.find("as3") != std::string::npos){
            goToPresetLocation(near_as3_);
            if (c_name.find("agv3") != std::string::npos){
                goToPresetLocation(at_agv3_as3_);
            }
            if (c_name.find("agv4") != std::string::npos){
                goToPresetLocation(at_agv4_as3_);
            }
        }
        if (c_name.find("as4") != std::string::npos){
            goToPresetLocation(home2_);
            goToPresetLocation(near_as4_);
            if (c_name.find("agv3") != std::string::npos){
                goToPresetLocation(at_agv3_as4_);
            }
            if (c_name.find("agv4") != std::string::npos){
                goToPresetLocation(at_agv4_as4_);
            }
        }
    }

    /////////////////////////////////////////////////////
    void Gantry::activateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        gantry_gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Gantry][activateGantryGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Gantry::deactivateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = false;
        gantry_gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Gantry][deactivateGantryGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    nist_gear::VacuumGripperState Gantry::getGripperState()
    {
        return gantry_gripper_state_;
    }

    /////////////////////////////////////////////////////
    void Gantry::goToPresetLocation(GantryPresetLocation location, bool full_robot)
    {
        ROS_INFO_STREAM("in preset");
        if (full_robot) {
            // gantry torso
            joint_group_positions_.at(0) = location.gantry_torso_preset.at(0);
            joint_group_positions_.at(1) = location.gantry_torso_preset.at(1);
            joint_group_positions_.at(2) = location.gantry_torso_preset.at(2);
            // gantry arm
            joint_group_positions_.at(3) = location.gantry_arm_preset.at(0);
            joint_group_positions_.at(4) = location.gantry_arm_preset.at(1);
            joint_group_positions_.at(5) = location.gantry_arm_preset.at(2);
            joint_group_positions_.at(6) = location.gantry_arm_preset.at(3);
            joint_group_positions_.at(7) = location.gantry_arm_preset.at(4);
            joint_group_positions_.at(8) = location.gantry_arm_preset.at(5);

            full_gantry_group_.setJointValueTarget(joint_group_positions_);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            // check a plan is found first then execute the action
            bool success = (full_gantry_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
                full_gantry_group_.move();
        }
        else {
            // gantry torso
            joint_group_positions_.at(0) = location.gantry_torso_preset.at(0);
            joint_group_positions_.at(1) = location.gantry_torso_preset.at(1);
            joint_group_positions_.at(2) = location.gantry_torso_preset.at(2);

            torso_gantry_group_.setJointValueTarget(joint_group_positions_);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            // check a plan is found first then execute the action
            bool success = (torso_gantry_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
                torso_gantry_group_.move();
        }

    }


    ///////////////////////////
    ////// Callback Functions
    ///////////////////////////


    /////////////////////////////////////////////////////
    void Gantry::gantry_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
    {
        gantry_gripper_state_ = *gripper_state_msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_full_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
    {
        if (joint_state_msg->position.size() == 0) {
            ROS_ERROR("[Gantry][gantry_full_joint_states_callback_] joint_state_msg->position.size() == 0!");
        }
        current_joint_states_ = *joint_state_msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        gantry_arm_controller_state_ = *msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        gantry_torso_controller_state_ = *msg;
    }
}//namespace