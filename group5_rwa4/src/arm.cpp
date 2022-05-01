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
        ROS_INFO_STREAM("In movepart");
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

        
        
        moveBaseTo(part_init_pose.position.y);

        // // move the arm above the part to grasp
        // // gripper stays at the current z
        // // only modify its x and y based on the part to grasp
        // // In this case we do not need to use preset locations
        // // everything is done dynamically
        // arm_ee_link_pose.position.x = part_init_pose.position.x;
        // arm_ee_link_pose.position.y = part_init_pose.position.y;
        // arm_ee_link_pose.position.z = arm_ee_link_pose.position.z;
        // // move the arm
        // arm_group_.setPoseTarget(arm_ee_link_pose);
        // arm_group_.move();

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
        postgrasp_pose3.position.z = arm_ee_link_pose.position.z;

        // preset z depending on the part type
        // some parts are bigger than others
        
        double z_pos{};
        if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.859;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.808;
        }
        if (part_type.find("regulator") != std::string::npos) {
            z_pos = 0.81;
        }
        if (part_type.find("battery") != std::string::npos) {
            z_pos = 0.79;
        }

        // flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        // arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        // arm_ee_link_pose.orientation.x = flat_orientation.getX();
        // arm_ee_link_pose.orientation.y = flat_orientation.getY();
        // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        // arm_ee_link_pose.orientation.w = flat_orientation.getW();

        
        // set of waypoints the arm will go through
        std::vector<geometry_msgs::Pose> waypoints;
        // pre-grasp pose: somewhere above the part
        auto pregrasp_pose = part_init_pose;
        pregrasp_pose.orientation = arm_ee_link_pose.orientation;
        pregrasp_pose.position.z = z_pos + 0.08;

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
    /////////////////////////////////////////////////////
    bool Arm::placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame, std::string agv)
    {
        goToPresetLocation(agv);
        // get the target pose of the part in the world frame
        auto target_pose_in_world = motioncontrol::transformtoWorldFrame(
            part_pose_in_frame,
            agv);


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
        // else if (location_name.compare("bin1") == 0) {
        //     location = bin1_;
        // }
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
    /////////////////////////////////////////////////////
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
    ///////////////////////////////
    std::vector<int>  Arm::pick_from_conveyor(std::vector<int> ebin, unsigned short int n)
    {   
        std::vector<int> rbin;
        int sbin = 0;
        for(auto &bin: ebin){
            ROS_INFO_STREAM("bin number "<< bin);
            if(bin == 1 || bin == 2 || bin == 5 || bin == 6)
            {
                sbin = bin;
                break;
            }
        }

        for(auto &bin: ebin){
            if(bin == sbin){
                continue;
            }
            else{
                rbin.push_back(bin);
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
            
            side_orientation = motioncontrol::quaternionFromEuler(0, 0, 0);
            arm_ee_link_pose = arm_group_.getCurrentPose().pose;
            arm_ee_link_pose.position.z = arm_ee_link_pose.position.z + 0.5; 
            arm_ee_link_pose.orientation.x = side_orientation.getX();
            arm_ee_link_pose.orientation.y = side_orientation.getY();
            arm_ee_link_pose.orientation.z = side_orientation.getZ();
            arm_ee_link_pose.orientation.w = side_orientation.getW();
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            
            ROS_INFO_STREAM("sbin number "<< sbin);
            geometry_msgs::Pose bin = get_part_pose_in_empty_bin(sbin);
            ROS_INFO_STREAM("Y_pos: "<< bin.position.y);
            side_orientation = motioncontrol::quaternionFromEuler(0, 0, 0);
            moveBaseTo(bin.position.y - 0.5);
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
        return rbin;
    }
    ///////////////////////////////
    void Arm::flippart(Product part, std::vector<int> rbin, geometry_msgs::Pose part_pose_in_frame, std::string agv){
        std::string part_type = part.type;
        geometry_msgs::Pose part_pose = part.world_pose;
        ROS_INFO_STREAM("In flip");
        pickPart(part_type, part_pose);
        int sbin = 0;
        for(auto &bin: rbin){
            ROS_INFO_STREAM("bin number "<< bin);
            if(bin == 1 || bin == 2 || bin == 5 || bin == 6)
            {
                sbin = bin;
                break;
            }
        }
        std::array<double,3> bin_origin{0,0,0};
        geometry_msgs::Pose part_world_pose;
        if (sbin == 1){
            bin_origin = bin1_origin_;
        }
        if (sbin == 2){
            bin_origin = bin2_origin_;
        }
        if (sbin == 3){
            bin_origin = bin3_origin_;
        }
        if (sbin == 4){
            bin_origin = bin4_origin_;
        }
        if (sbin == 5){
            bin_origin = bin5_origin_;
        }
        if (sbin == 6){
            bin_origin = bin6_origin_;
        }
        if (sbin == 7){
            bin_origin = bin7_origin_;
        }
        if (sbin == 8){
            bin_origin = bin8_origin_;
        }
        ROS_INFO_STREAM("EMPTYBIN: "<<sbin);
        moveBaseTo(bin_origin.at(1));
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

        part_pose.position.x = arm_ee_link_pose.position.x;
        part_pose.position.y = arm_ee_link_pose.position.y;
        part_pose.position.z = 0.8;
        part_pose.orientation.x = target_pose.getX();
        part_pose.orientation.y = target_pose.getY();
        part_pose.orientation.z = target_pose.getZ();
        part_pose.orientation.w = target_pose.getW();

        arm_ee_link_pose.position.z = bin_origin.at(2)+0.3;
        geometry_msgs::Pose Post_grasp = arm_ee_link_pose;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(Post_grasp);
        arm_group_.move();
        
        auto side_orientation = motioncontrol::quaternionFromEuler(0, 0, 1.57);
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
        arm_ee_link_pose.position.y = part_pose.position.y-0.12 ;
        arm_ee_link_pose.position.z = part_pose.position.z;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        while (!gripper_state_.attached) {
            arm_ee_link_pose.position.y += 0.005;
            arm_group_.setPoseTarget(arm_ee_link_pose);
            arm_group_.move();
            ros::Duration(sleep(0.5));
        }

        arm_ee_link_pose.position.z =arm_ee_link_pose.position.z+0.2;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        
        arm_ee_link_pose.position.x = bin_origin.at(0);
        arm_ee_link_pose.position.y = bin_origin.at(1)-0.07;
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();
        
        // arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        // tf2::Quaternion qn_current(
        //     arm_ee_link_pose.orientation.x,
        //     arm_ee_link_pose.orientation.y,
        //     arm_ee_link_pose.orientation.z,
        //     arm_ee_link_pose.orientation.w);
        // target_pose = motioncontrol::quaternionFromEuler(3.14, 0, 1.57);
        // tf2::Quaternion n_target_part(
        //     target_pose.getX(),
        //     target_pose.getY(),
        //     target_pose.getZ(),
        //     target_pose.getW());
        // q_rot = n_target_part * q_target_part.inverse();
        // q_rslt = q_rot * qn_current;
        // q_rslt.normalize();
        // arm_ee_link_pose.orientation.x = q_rslt.x();
        // arm_ee_link_pose.orientation.y = q_rslt.y();
        // arm_ee_link_pose.orientation.z = q_rslt.z();
        // arm_ee_link_pose.orientation.w = q_rslt.w();
        // arm_group_.setMaxVelocityScalingFactor(1.0);
        // arm_group_.setPoseTarget(arm_ee_link_pose);
        // arm_group_.move();
        // ros::Duration(2.0).sleep();
        // deactivateGripper();
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

namespace gantry {
    /////////////////////////////////////////////////////
    Arm::Arm(ros::NodeHandle& node) : node_("/ariac/gantry"),
        planning_group_("/ariac/gantry/robot_description"),
        arm_options_("gantry_full", planning_group_, node_),
        arm_group_(arm_options_)
    {
        ROS_INFO_STREAM("[Gantry Arm] constructor called... ");

    }

    /////////////////////////////////////////////////////
    void Arm::init()
    {
        // make sure the planning group operates in the world frame
        // check the name of the end effector
        // ROS_INFO_NAMED("init", "End effector link: %s", arm_group_.getEndEffectorLink().c_str());


        // publishers to directly control the joints without moveit
        arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);
        // joint state subscribers
        arm_joint_states_subscriber_ =
            node_.subscribe("/ariac/gantry/joint_states", 10, &Arm::arm_joint_states_callback_, this);
        // controller state subscribers
        arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_arm_controller/state", 10, &Arm::arm_controller_state_callback, this);
        // gripper state subscriber
        gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/arm/gripper/state", 10, &Arm::gripper_state_callback, this);
        // controller state subscribers
        gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
        gripper_control_client_.waitForExistence();


        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the arm are in this order:
        // - gantry_arm_shoulder_pan_joint
        // - gantry_arm_shoulder_lift_joint
        // - gantry_arm_elbow_joint
        // - gantry_arm_wrist_1_joint
        // - gantry_arm_wrist_2_joint
        // - gantry_arm_wrist_3_joint
        // - small_long_joint
        // - torso_base_main_joint
        // - torso_rail_joint

        double gantry_arm_shoulder_pan_joint{ 0 };
        double gantry_arm_shoulder_lift_joint{ -0.92 };
        double gantry_arm_elbow_joint{ 1.20 };
        double gantry_arm_wrist_1_joint{ -0.25 };
        double gantry_arm_wrist_2_joint{ 1.54 };
        double gantry_arm_wrist_3_joint{ 0.83 };
        double small_long_joint{ -2.45 };
        double torso_rail_joint{ 0 };
        double torso_base_main_joint{ -1.57 };
        

        //home position
        home1_.arm_preset = { -2.45, 0.0, -1.57, -0,.01 -0.92, 1.20, -0.25, 1.54, 0.83 };
        home1_.name = "home1";
        home2_.arm_preset = { -1.01, 0.0, -1.57, -0,.01 -0.92, 1.20, -0.25, 1.54, 0.83 };
        home2_.name = "home2";
        agv1_.arm_preset = { -2.45, -3.83, -1.57, -0,.01 -0.92, 1.20, -0.25, 1.54, 0.83 };
        agv1_.name = "agv1";
        agv2_.arm_preset = { -2.45, -1.83, -1.57, -0,.01 -0.92, 1.20, -0.25, 1.54, 0.83 };
        agv2_.name = "agv2";
        agv3_.arm_preset = { -2.45, 1.83, -1.57, -0,.01 -0.92, 1.20, -0.25, 1.54, 0.83 };
        agv3_.name = "agv3";
        agv4_.arm_preset = { -2.45, 4.33, -1.57, -0,.01 -0.92, 1.20, -0.25, 1.54, 0.83 };
        agv4_.name = "agv4";


        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("gantry_full");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();
        // next get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);
    }

    //////////////////////////////////////////////////////
    void Arm::moveBaseTo(double torso_rail_joint) {
        // get the current joint positions
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("gantry_full");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();

        // get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

        // next, assign a value to only the linear_arm_actuator_joint
        joint_group_positions_.at(1) = -torso_rail_joint;

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

        
        
        moveBaseTo(part_init_pose.position.y);

        // // move the arm above the part to grasp
        // // gripper stays at the current z
        // // only modify its x and y based on the part to grasp
        // // In this case we do not need to use preset locations
        // // everything is done dynamically
        // arm_ee_link_pose.position.x = part_init_pose.position.x;
        // arm_ee_link_pose.position.y = part_init_pose.position.y;
        // arm_ee_link_pose.position.z = arm_ee_link_pose.position.z;
        // // move the arm
        // arm_group_.setPoseTarget(arm_ee_link_pose);
        // arm_group_.move();

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
        postgrasp_pose3.position.z = arm_ee_link_pose.position.z;

        // preset z depending on the part type
        // some parts are bigger than others
        
        double z_pos{};
        if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.859;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.81;
        }
        if (part_type.find("regulator") != std::string::npos) {
            z_pos = 0.81;
        }
        if (part_type.find("battery") != std::string::npos) {
            z_pos = 0.79;
        }

        // flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        // arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        // arm_ee_link_pose.orientation.x = flat_orientation.getX();
        // arm_ee_link_pose.orientation.y = flat_orientation.getY();
        // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        // arm_ee_link_pose.orientation.w = flat_orientation.getW();

        
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


    /////////////////////////////////////////////////////
    bool Arm::placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame, std::string agv)
    {
        goToPresetLocation(agv);
        // get the target pose of the part in the world frame
        auto target_pose_in_world = motioncontrol::transformtoWorldFrame(
            part_pose_in_frame,
            agv);


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
        goToPresetLocation("home1");

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
        //make sure the planning groups operated in the world frame
        // ROS_INFO_NAMED("init", "Planning frame: %s", full_gantry_group_.getPlanningFrame().c_str());
        // ROS_INFO_NAMED("init", "Planning frame: %s", arm_gantry_group_.getPlanningFrame().c_str());
        // ROS_INFO_NAMED("init", "Planning frame: %s", torso_gantry_group_.getPlanningFrame().c_str());

        //check the name of the end effector
        // ROS_INFO_NAMED("init", "End effector link: %s", full_gantry_group_.getEndEffectorLink().c_str());
        // ROS_INFO_NAMED("init", "End effector link: %s", arm_gantry_group_.getEndEffectorLink().c_str());


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

        home2_.gantry_torso_preset = { -8.5, 0.0, -1.57 };
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
        at_bin2_.gantry_torso_preset = { 0.0, -3.35, 3.14 };
        at_bin2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin2_.gantry_full_preset.insert(at_bin2_.gantry_full_preset.begin(), at_bin2_.gantry_torso_preset.begin(), at_bin2_.gantry_torso_preset.end());
        at_bin2_.gantry_full_preset.insert(at_bin2_.gantry_full_preset.end(), at_bin2_.gantry_arm_preset.begin(), at_bin2_.gantry_arm_preset.end());

        // above bin3
        at_bin3_.gantry_torso_preset = { -0.78, -3.35, 3.14 };
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
        at_bin8_.gantry_torso_preset = { -0.74, 2.63, 3.14 };
        at_bin8_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin8_.gantry_full_preset.insert(at_bin8_.gantry_full_preset.begin(), at_bin8_.gantry_torso_preset.begin(), at_bin8_.gantry_torso_preset.end());
        at_bin8_.gantry_full_preset.insert(at_bin8_.gantry_full_preset.end(), at_bin8_.gantry_arm_preset.begin(), at_bin8_.gantry_arm_preset.end());

        // above agv3_as3
        at_agv3_as3_.gantry_torso_preset = { -3.40, 2.10, 0.0 };
        at_agv3_as3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv3_as3_.gantry_full_preset.insert(at_agv3_as3_.gantry_full_preset.begin(), at_agv3_as3_.gantry_torso_preset.begin(), at_agv3_as3_.gantry_torso_preset.end());
        at_agv3_as3_.gantry_full_preset.insert(at_agv3_as3_.gantry_full_preset.end(), at_agv3_as3_.gantry_arm_preset.begin(), at_agv3_as3_.gantry_arm_preset.end());

        // above agv4_as3
        at_agv4_as3_.gantry_torso_preset = { -3.70, 3.99, 3.14 };
        at_agv4_as3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv4_as3_.gantry_full_preset.insert(at_agv4_as3_.gantry_full_preset.begin(), at_agv4_as3_.gantry_torso_preset.begin(), at_agv4_as3_.gantry_torso_preset.end());
        at_agv4_as3_.gantry_full_preset.insert(at_agv4_as3_.gantry_full_preset.end(), at_agv4_as3_.gantry_arm_preset.begin(), at_agv4_as3_.gantry_arm_preset.end());

        // above near_as3
        near_as3_.gantry_torso_preset = { -3.30, 2.91, 1.57 };
        near_as3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as3_.gantry_full_preset.insert(near_as3_.gantry_full_preset.begin(), near_as3_.gantry_torso_preset.begin(), near_as3_.gantry_torso_preset.end());
        near_as3_.gantry_full_preset.insert(near_as3_.gantry_full_preset.end(), near_as3_.gantry_arm_preset.begin(), near_as3_.gantry_arm_preset.end());

        // above as3
        at_as3_.gantry_torso_preset = { -3.85, 2.82, 1.57 };
        at_as3_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_as3_.gantry_full_preset.insert(at_as3_.gantry_full_preset.begin(), at_as3_.gantry_torso_preset.begin(), at_as3_.gantry_torso_preset.end());
        at_as3_.gantry_full_preset.insert(at_as3_.gantry_full_preset.end(), at_as3_.gantry_arm_preset.begin(), at_as3_.gantry_arm_preset.end());

        // above agv1_as1
        at_agv1_as1_.gantry_torso_preset = { -3.55, -3.93, 0.0 };
        at_agv1_as1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv1_as1_.gantry_full_preset.insert(at_agv1_as1_.gantry_full_preset.begin(), at_agv1_as1_.gantry_torso_preset.begin(), at_agv1_as1_.gantry_torso_preset.end());
        at_agv1_as1_.gantry_full_preset.insert(at_agv1_as1_.gantry_full_preset.end(), at_agv1_as1_.gantry_arm_preset.begin(), at_agv1_as1_.gantry_arm_preset.end());

        // above agv2_as1
        at_agv2_as1_.gantry_torso_preset = { -3.7, -2.13, 3.14 };
        at_agv2_as1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv2_as1_.gantry_full_preset.insert(at_agv2_as1_.gantry_full_preset.begin(), at_agv2_as1_.gantry_torso_preset.begin(), at_agv2_as1_.gantry_torso_preset.end());
        at_agv2_as1_.gantry_full_preset.insert(at_agv2_as1_.gantry_full_preset.end(), at_agv2_as1_.gantry_arm_preset.begin(), at_agv2_as1_.gantry_arm_preset.end());

        // above near_as1
        near_as1_.gantry_torso_preset = { -3.0, -3.21, 3.14 };
        near_as1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as1_.gantry_full_preset.insert(near_as1_.gantry_full_preset.begin(), near_as1_.gantry_torso_preset.begin(), near_as1_.gantry_torso_preset.end());
        near_as1_.gantry_full_preset.insert(near_as1_.gantry_full_preset.end(), near_as1_.gantry_arm_preset.begin(), near_as1_.gantry_arm_preset.end());

        // above as1
        at_as1_.gantry_torso_preset = { -3.65, -3.21, 1.57 };
        at_as1_.gantry_arm_preset = { 0 , -2.26 , 1.50 ,0.76 ,1.55 ,0.83 };
        at_as1_.gantry_full_preset.insert(at_as1_.gantry_full_preset.begin(), at_as1_.gantry_torso_preset.begin(), at_as1_.gantry_torso_preset.end());
        at_as1_.gantry_full_preset.insert(at_as1_.gantry_full_preset.end(), at_as1_.gantry_arm_preset.begin(), at_as1_.gantry_arm_preset.end());

        // above agv1_as2
        at_agv1_as2_.gantry_torso_preset = { -8.55, -3.93, 0.0 };
        at_agv1_as2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv1_as2_.gantry_full_preset.insert(at_agv1_as2_.gantry_full_preset.begin(), at_agv1_as2_.gantry_torso_preset.begin(), at_agv1_as2_.gantry_torso_preset.end());
        at_agv1_as2_.gantry_full_preset.insert(at_agv1_as2_.gantry_full_preset.end(), at_agv1_as2_.gantry_arm_preset.begin(), at_agv1_as2_.gantry_arm_preset.end());

        // above agv2_as2
        at_agv2_as2_.gantry_torso_preset = { -8.7, -2.13, 3.14 };
        at_agv2_as2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv2_as2_.gantry_full_preset.insert(at_agv2_as2_.gantry_full_preset.begin(), at_agv2_as2_.gantry_torso_preset.begin(), at_agv2_as2_.gantry_torso_preset.end());
        at_agv2_as2_.gantry_full_preset.insert(at_agv2_as2_.gantry_full_preset.end(), at_agv2_as2_.gantry_arm_preset.begin(), at_agv2_as2_.gantry_arm_preset.end());

        // above near_as2
        near_as2_.gantry_torso_preset = { -8.30, -3.21, 1.57 };
        near_as2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as2_.gantry_full_preset.insert(near_as2_.gantry_full_preset.begin(), near_as2_.gantry_torso_preset.begin(), near_as2_.gantry_torso_preset.end());
        near_as2_.gantry_full_preset.insert(near_as2_.gantry_full_preset.end(), near_as2_.gantry_arm_preset.begin(), near_as2_.gantry_arm_preset.end());

        // above as2
        at_as2_.gantry_torso_preset = { -8.85, -3.21, 1.57 };
        at_as2_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_as2_.gantry_full_preset.insert(at_as2_.gantry_full_preset.begin(), at_as2_.gantry_torso_preset.begin(), at_as2_.gantry_torso_preset.end());
        at_as2_.gantry_full_preset.insert(at_as2_.gantry_full_preset.end(), at_as2_.gantry_arm_preset.begin(), at_as2_.gantry_arm_preset.end());

        // above agv3_as4
        at_agv3_as4_.gantry_torso_preset = { -8.40, 2.10, 0.0 };
        at_agv3_as4_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv3_as4_.gantry_full_preset.insert(at_agv3_as4_.gantry_full_preset.begin(), at_agv3_as4_.gantry_torso_preset.begin(), at_agv3_as4_.gantry_torso_preset.end());
        at_agv3_as4_.gantry_full_preset.insert(at_agv3_as4_.gantry_full_preset.end(), at_agv3_as4_.gantry_arm_preset.begin(), at_agv3_as4_.gantry_arm_preset.end());

        // above agv4_as4
        at_agv4_as4_.gantry_torso_preset = { -8.70, 3.99, 3.14 };
        at_agv4_as4_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv4_as4_.gantry_full_preset.insert(at_agv4_as4_.gantry_full_preset.begin(), at_agv4_as4_.gantry_torso_preset.begin(), at_agv4_as4_.gantry_torso_preset.end());
        at_agv4_as4_.gantry_full_preset.insert(at_agv4_as4_.gantry_full_preset.end(), at_agv4_as4_.gantry_arm_preset.begin(), at_agv4_as4_.gantry_arm_preset.end());

        // above near_as4
        near_as4_.gantry_torso_preset = { -8.30, 2.91, 1.57 };
        near_as4_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        near_as4_.gantry_full_preset.insert(near_as4_.gantry_full_preset.begin(), near_as4_.gantry_torso_preset.begin(), near_as4_.gantry_torso_preset.end());
        near_as4_.gantry_full_preset.insert(near_as4_.gantry_full_preset.end(), near_as4_.gantry_arm_preset.begin(), near_as4_.gantry_arm_preset.end());

        // above as4
        at_as4_.gantry_torso_preset = { -8.85, 2.82, 1.57 };
        at_as4_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_as4_.gantry_full_preset.insert(at_as4_.gantry_full_preset.begin(), at_as4_.gantry_torso_preset.begin(), at_as4_.gantry_torso_preset.end());
        at_as4_.gantry_full_preset.insert(at_as4_.gantry_full_preset.end(), at_as4_.gantry_arm_preset.begin(), at_as4_.gantry_arm_preset.end());


        // before approaching agv1
        // before_agv1_.gantry_torso = { 0.07, -2.92, -0.02 };
        // before_agv1_.gantry_arm = { 0.07 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        // before_agv1_.gantry_full.insert(before_agv1_.gantry_full.begin(), before_agv1_.gantry_torso.begin(), before_agv1_.gantry_torso.end());
        // before_agv1_.gantry_full.insert(before_agv1_.gantry_full.end(), before_agv1_.gantry_arm.begin(), before_agv1_.gantry_arm.end());

        // above agv1
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv1_.gantry_torso_preset = { -0.37, -3.78, -0.69 };
        at_agv1_.gantry_arm_preset = { -0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        at_agv1_.gantry_full_preset.insert(at_agv1_.gantry_full_preset.begin(), at_agv1_.gantry_torso_preset.begin(), at_agv1_.gantry_torso_preset.end());
        at_agv1_.gantry_full_preset.insert(at_agv1_.gantry_full_preset.end(), at_agv1_.gantry_arm_preset.begin(), at_agv1_.gantry_arm_preset.end());

        // above agv2
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv2_.gantry_torso_preset = { -0.37, -2.16, -2.70 };
        at_agv2_.gantry_arm_preset = { -0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        at_agv2_.gantry_full_preset.insert(at_agv2_.gantry_full_preset.begin(), at_agv2_.gantry_torso_preset.begin(), at_agv2_.gantry_torso_preset.end());
        at_agv2_.gantry_full_preset.insert(at_agv2_.gantry_full_preset.end(), at_agv2_.gantry_arm_preset.begin(), at_agv2_.gantry_arm_preset.end());

        // above agv3
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv3_.gantry_torso_preset = { -0.37, 2.41, -0.69 };
        at_agv3_.gantry_arm_preset = { -0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
        at_agv3_.gantry_full_preset.insert(at_agv3_.gantry_full_preset.begin(), at_agv3_.gantry_torso_preset.begin(), at_agv3_.gantry_torso_preset.end());
        at_agv3_.gantry_full_preset.insert(at_agv3_.gantry_full_preset.end(), at_agv3_.gantry_arm_preset.begin(), at_agv3_.gantry_arm_preset.end());

        // above agv4
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv4_.gantry_torso_preset = { -0.37, 3.49, -2.70 };
        at_agv4_.gantry_arm_preset = { -0.01, -0.92, 1.20, -0.25, 1.54, 0.83 };
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
    bool Gantry::pickPart(geometry_msgs::Pose part_init_pose)
    {
        ros::Duration(3).sleep();
        activateGripper();
        const double GRIPPER_HEIGHT = 0.01;
        const double EPSILON = 0.008; // for the gripper to firmly touch
        ros::Duration(0.5).sleep();

        // pose of the end effector in the world frame
        geometry_msgs::Pose gantry_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        tf2::Quaternion  gantry_ee_link_orientation(
            gantry_ee_link_pose.orientation.x,
            gantry_ee_link_pose.orientation.y,
            gantry_ee_link_pose.orientation.z,
            gantry_ee_link_pose.orientation.w
        );

        // pose to go to after grasping a part (lift the arm a little bit)
        geometry_msgs::Pose postGraspPose;
        auto part_init_pose_in_world = part_init_pose;

        part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + 0.06;
        part_init_pose_in_world.orientation.x = gantry_ee_link_pose.orientation.x;
        part_init_pose_in_world.orientation.y = gantry_ee_link_pose.orientation.y;
        part_init_pose_in_world.orientation.z = gantry_ee_link_pose.orientation.z;
        part_init_pose_in_world.orientation.w = gantry_ee_link_pose.orientation.w;

        // activate gripper
        ros::Duration(3).sleep();
        activateGripper();
        auto state = getGripperState();


        if (!state.enabled) {
            int MAX_ATTEMPT = 10;
            for (int i{}; i < MAX_ATTEMPT; i++) {
                ros::Duration(0.5).sleep();
                activateGripper();
                state = getGripperState();
            }
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
            postGraspPose.position.z = postGraspPose.position.z + 0.1;
            //--Move arm to previous position
            arm_gantry_group_.setPoseTarget(postGraspPose);
            arm_gantry_group_.move();
            ros::Duration(2.0).sleep();
            arm_gantry_group_.setPoseTarget(gantry_ee_link_pose);
            arm_gantry_group_.move();
            return true;
        }

        // if (state.enabled) {
        //     ROS_INFO_STREAM("[Gripper] = enabled");
        //     //--Move arm to part
        //     full_gantry_group_.setPoseTarget(part_init_pose_in_world);
        //     moveit::planning_interface::MoveGroupInterface::Plan plan;
        //     full_gantry_group_.plan(plan);
        //     auto plan_result = full_gantry_group_.execute(plan);//TODO catch the error message

        //     state = getGripperState();
        //     // move the arm closer until the object is attached
        //     while (!state.attached) {
        //         part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
        //         full_gantry_group_.setPoseTarget(part_init_pose_in_world);
        //         full_gantry_group_.move();
        //         full_gantry_group_.setPoseTarget(gantry_ee_link_pose);
        //         state = getGripperState();
        //     }

        //     ROS_INFO_STREAM("[Gripper] = object attached");
        //     // ros::Duration(1.0).sleep();
        //     postGraspPose = full_gantry_group_.getCurrentPose().pose;
        //     postGraspPose.position.z = postGraspPose.position.z + 0.1;
        //     //--Move arm to previous position
        //     full_gantry_group_.setPoseTarget(postGraspPose);
        //     full_gantry_group_.move();
        //     ros::Duration(2.0).sleep();
        //     full_gantry_group_.setPoseTarget(gantry_ee_link_pose);
        //     full_gantry_group_.move();
        //     return true;
        // }
        return false;
    }


    /////////////////////////////////////////////////////
    bool Gantry::placePart(geometry_msgs::Pose part_init_pose_in_world, geometry_msgs::Pose target_pose_in_frame, std::string location)
    {
        // get the target pose of the part in the world frame
        auto part_in_world_frame = motioncontrol::transformtoWorldFrame(
            target_pose_in_frame,
            location);

        // // complete our struct instance
        // geometry_msgs::Pose target_pose_in_world;
        // target_pose_in_world.position.x = part_in_world_frame.position.x;
        // target_pose_in_world.position.y = part_in_world_frame.position.y;
        // target_pose_in_world.position.z = part_in_world_frame.position.z;
        // target_pose_in_world.orientation.x = part_in_world_frame.orientation.x;
        // target_pose_in_world.orientation.y = part_in_world_frame.orientation.y;
        // target_pose_in_world.orientation.z = part_in_world_frame.orientation.z;
        // target_pose_in_world.orientation.w = part_in_world_frame.orientation.w;
        // motioncontrol::print(target_pose_in_world);

        // geometry_msgs::Pose currentPose = full_gantry_group_.getCurrentPose().pose;

        if (location == "agv1") {
            goToPresetLocation(at_agv1_);
        }
        if (location == "agv2") {
            goToPresetLocation(at_agv2_);
        }
        if (location == "agv3") {
            goToPresetLocation(at_agv3_);
        }
        if (location == "agv4") {
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
            part_in_world_frame.position.x,
            part_in_world_frame.position.y,
            part_in_world_frame.position.z);

        ROS_INFO("Target World Orientation: %f, %f, %f, %f",
            part_in_world_frame.orientation.x,
            part_in_world_frame.orientation.y,
            part_in_world_frame.orientation.z,
            part_in_world_frame.orientation.w);

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
            part_in_world_frame.orientation.x,
            part_in_world_frame.orientation.y,
            part_in_world_frame.orientation.z,
            part_in_world_frame.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        // orientation of the gripper when placing the part in the tray
        part_in_world_frame.orientation.x = q_rslt.x();
        part_in_world_frame.orientation.y = q_rslt.y();
        part_in_world_frame.orientation.z = q_rslt.z();
        part_in_world_frame.orientation.w = q_rslt.w();
        part_in_world_frame.position.z += 0.1;

        //allow replanning if it fails
        // arm_gantry_group_.allowReplanning(true);
        //Set the allowable error of position (unit: meter) and attitude (unit: radians)
        // arm_gantry_group_.setGoalPositionTolerance(0.001);
        // arm_gantry_group_.setGoalOrientationTolerance(0.01);
        // //Set the maximum speed and acceleration allowed
        // arm_gantry_group_.setMaxAccelerationScalingFactor(0.2);
        // arm_gantry_group_.setMaxVelocityScalingFactor(0.2);

        arm_gantry_group_.setPoseTarget(part_in_world_frame);
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