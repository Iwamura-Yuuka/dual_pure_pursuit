#ifndef DUAL_PURE_PURSUIT_PLANNER_H
#define DUAL_PURE_PURSUIT_PLANNER_H

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h> 
#include<geometry_msgs/PoseStamped.h>
#include<tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Pose2D.h>

#include<ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include<dynamixel_workbench_msgs/DynamixelStateList.h>

class DualPurePursuitPlanner
{
public:
  DualPurePursuitPlanner();
  void process();

private:
  // コールバック関数
  void predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg);
  void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

  // 引数なし関数
  bool is_goal();                                                                           // goalに着くまでfalseを返す
  void update_carrots();
  void update_motion();

  // 引数あり関数
  double calc_dist(const double &x1, const double &y1, const double &x2, const double &y2);
  double calc_steering_angle(double r, double delta, double a, char side);
  double calc_turn_radius(double r, double delta, double a, char side);
  void flip_velocity(double &delta, double &velocity);


  //yamlファイルで設定可能な変数
  int hz_;
  double dist_to_carrot1_;  // carrot1（cmd_vel計算用ターゲット）までの距離 [m]
  double dist_to_carrot2_;  // carrot2（ステア角計算用ターゲット）までの距離 [m]
  double max_target_velocity_;
  double max_yawrate_;      // 最高旋回速度 [rad/s]
  double max_steer_angle_;
  double tread_;
  double pitch_offset_;
  double goal_tolerance_;

  // msgの受け取り判定用
  bool flag_predicted_path_ = false;
  bool flag_robot_odom_ = false;

  // 座標変換の判定用
  bool flag_frame_change_ = false;
  


  //member
  geometry_msgs::Pose carrot1_, carrot2_;
  double carrot_distance_;
  double target_velocity_;

  //NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;


  //Subscriber
  ros::Subscriber sub_predicted_path_;
  ros::Subscriber sub_robot_odom_;
  
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_cmd_pos_;

  // tf
  tf2_ros::Buffer tf_buffer_;

  
  geometry_msgs::Pose2D current_pose_;
  std::string world_frame_id_;
  std::string robot_frame_id_;

  //debug
  ros::Publisher pub_carrot1_;
  ros::Publisher pub_carrot2_;
  nav_msgs::Path cx_line_;

  // 各種オブジェクト
  nav_msgs::Path predicted_path_;            // 目標軌跡
  nav_msgs::Odometry robot_odom_;         // ロボットの位置情報


};

#endif  // DUAL_PURE_PURSUIT_PLANNER_H 