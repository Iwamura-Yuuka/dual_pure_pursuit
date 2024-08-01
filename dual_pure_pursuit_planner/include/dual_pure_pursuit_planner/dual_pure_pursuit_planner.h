#ifndef DUAL_PURE_PURSUIT_PLANNER_H
#define DUAL_PURE_PURSUIT_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

class DualPurePursuitPlanner
{
public:
  DualPurePursuitPlanner();
  void process();

private:
  //コールバック関数
  void target_path_callback(const nav_msgs::Path::ConstPtr& msg);
  void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  
  //引数なし関数
  void update_carrots();  // ターゲットのキャロットを更新
  void update_motion();   // 制御入力を計算

  // 引数あり関数
  double calc_dist(const double x1, const double y1, const double x2, const double y2);  // 距離を計算
  void calc_carrot(geometry_msgs::PointStamped& carrot, const double dist_to_carrot);    // ターゲットのキャロットの位置を計算
  void transform_carrot(geometry_msgs::PointStamped& carrot);                            // キャロットをodom座標系からbase_footprint座標系に変換

  //yamlファイルで設定可能な変数
  bool visualize_for_debug_;  // carrotを可視化するかの設定用
  bool use_global_path_;          // global_pathをtarget_pathとするときはtrue
  int hz_;                            //ループ周波数[Hz]
  double dist_to_carrot1_;  // carrot1（cmd_vel計算用ターゲット）までの距離[m]
  double dist_to_carrot2_;  // carrot2（ステア角計算用ターゲット）までの距離[m]
  std::string world_frame_;  // 目標軌道のframe_id
  std::string robot_frame_;  // ロボットのframe_id（base_footprint）

  // global_pathをtarget_pathとするとき用変数
  int tmp_carrot1_index_;  // carrot1のindex格納用
  int tmp_carrot2_index_;  // carrot2のindex格納用

  //msgの受け取り判定用
  bool flag_target_path_ = false;
  bool flag_robot_odom_ = false;

  // 座標変換の判定用
  bool flag_frame_change_ = false;

  //NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  //Subscriber
  ros::Subscriber sub_target_path_;
  ros::Subscriber sub_robot_odom_;

  //Publisher
  ros::Publisher pub_carrot1_;
  ros::Publisher pub_carrot2_;

  // tf
  tf2_ros::Buffer tf_buffer_;

  // 各種オブジェクト
  nav_msgs::Path target_path_;              // 目標軌道
  nav_msgs::Odometry robot_odom_;       // ロボットの位置情報
  geometry_msgs::PointStamped carrot1_;  // cmd_vel計算用ターゲット
  geometry_msgs::PointStamped carrot2_;  // ステア角計算用ターゲット
};

#endif  // DUAL_PURE_PURSUIT_PLANNER_H 