#ifndef DUAL_PURE_PURSUIT_PLANNER_H
#define DUAL_PURE_PURSUIT_PLANNER_H

#include<ros/ros.h>
#include<ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include<nav_msgs/Path.h>  // Express velocity in terms of linear and angular components , Vector3
#include<geometry_msgs/Twist.h>
#include<dynamixel_workbench_msgs/DynamixelStateList.h>  // "dynamixel_state" is list type
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Pose2D.h>

class DualPurePursuitPlanner
{
public:
    DualPurePursuitPlanner();
    ~DualPurePursuitPlanner();
    void process();

private:
    int hz_;
    double L1_, L2_;  //L1 is distance to the first target point , L2 is distance to the second target point
    double MAX_TARGET_VELOCITY_;
    double MAX_STEERING_ANGLE_;
    double TREAD_;
    double PITCH_OFFSET_;
    double GOAL_BORDER_;

    void predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg);
    void trajectory_marker_callback(const visualization_msgs::Marker::ConstPtr &msg);
    void update_carrots();
    void update_motion();
    void update_target_velocity();
    double calc_distance(const double &x1, const double &y1, const double &x2, const double &y2);
    double calc_steering_angle(double r, double delta, double a, char side);
    double calc_turn_radius(double r, double delta, double a, char side);
    void flip_velocity(double &delta, double &velocity);

    //member
    nav_msgs::Path predicted_path_;
    geometry_msgs::Pose carrot1_, carrot2_;
    double carrot_distance_;
    double target_velocity_;
    bool have_received_path_=false;
    bool have_reached_goal_=false;
    bool read_marker_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_local_goal_;
    ros::Subscriber sub_marker_;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_cmd_pos_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::Pose2D current_pose_;
    std::string world_frame_id_;
    std::string robot_frame_id_;

    //debug
    ros::Publisher pub_carrot1_;
    ros::Publisher pub_carrot2_;
    ros::Publisher pub_cx_line_;
    nav_msgs::Path cx_line_;
};

#endif  // DUAL_PURE_PURSUIT_PLANNER_H 