#ifndef DUAL_PURE_PURSUIT_PLANNER_H
#define DUAL_PURE_PURSUIT_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include<ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include<dynamixel_workbench_msgs/DynamixelStateList.h> 

// ===== 構造体 =====
struct State
{
  double x;        // [m]
  double y;        // [m]
  double yaw;      // [rad]
  double velocity; // [m/s]
  double yawrate;  // [rad/s]
};

// ===== クラス =====
class DualPurePursuitPlanner
{
public:
  DualPurePursuitPlanner();
  void process();

private:
  //コールバック関数
  void target_path_callback(const nav_msgs::Path::ConstPtr& msg);
  void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  //引数なし関数
  bool is_goal();                                                                                       // goalに着くまでtrueを返す

  // 引数あり関数
  double calc_dist(const double x1, const double y1, const double x2, const double y2);                 // 距離を計算
  // void calc_carrot(geometry_msgs::PointStamped& carrot, const double dist_to_carrot);                   // ターゲットのキャロットの位置を計算
  // void transform_carrot(geometry_msgs::PointStamped& carrot);                                           // キャロットをodom座標系からbase_footprint座標系に変換
  void update_carrots(ros::Time now);                                                                   // ターゲットのキャロットを更新
  double normalize_angle(double theta);                                                                 // 適切な角度(-M_PI~M_PI)に変換
  void move_image(State& imstate, const double velocity, const double yawrate, const double delta);     // 仮想ロボットを移動
  std::vector<State> calc_trajectory(const double velocity, const double yawrate, const double delta);  // 予測軌跡を作成
  double calc_evaluation(const std::vector<State>& trajectory);                                         // 評価関数を計算
  bool is_in_map(const double x, const double y);                                                       // マップ内の場合，trueを返す
  int xy_to_grid_index(const double x, const double y);                                                 // 座標からグリッドのインデックスを返す
  double calc_obs_score(const std::vector<State>& trajectory);                                          // obsの評価関数を計算（障害物に衝突するか）
  double calc_vel_score(const std::vector<State>& trajectory);                                          // 並進速度の評価関数を計算
  double calc_steering_angle(const double r, const double delta, const double d, const char side);      // ステア角を計算
  double fix_steer_angle(const double r, const double delta_inside, const double d, int sign);          // ステア角の最大値に基づき修正
  void flip_velocity(double &delta, double &velocity);                                                  // ステア角に合わせて速度を反転
  void update_motion(ros::Time now);                                                                    // 制御入力を計算
  void visualize_trajectory(std::vector<State>& trajectory, ros::Time now);                             // 軌跡を可視化

  //yamlファイルで設定可能な変数
  bool visualize_for_debug_;    // carrotを可視化するかの設定用
  bool use_global_path_;        // global_pathをtarget_pathとするときはtrue
  int hz_;                      //ループ周波数 [Hz]
  double max_target_velocity_;  // 目標並進速度 [m/s]
  double max_steer_angle_;         // ステア角の最大値 [deg]
  double tread_;                        // CCVのトレッド幅 [m]
  double dist_to_carrot1_;  // carrot1（cmd_vel計算用ターゲット）までの距離 [m]
  double dist_to_carrot2_;  // carrot2（ステア角計算用ターゲット）までの距離 [m]
  double goal_tolerance_;      // ゴールに対する許容誤差 [m]
  std::string world_frame_;  // 目標軌道のframe_id
  std::string robot_frame_;  // ロボットのframe_id（base_footprint）
  double max_yawrate_;      // 最高旋回速度 [rad/s]
  double vel_reso_;                // 並進速度を計算するときの刻み幅 [m/s]
  double dt_;       // 微小時間 [s]
  double predict_time_;  // 予測時間 [s]

  // global_pathをtarget_pathとするとき用変数
  int tmp_carrot1_index_;  // carrot1のindex格納用
  int tmp_carrot2_index_;  // carrot2のindex格納用

  //msgの受け取り判定用
  bool flag_target_path_ = false;
  bool flag_robot_odom_ = false;
  bool flag_local_map_ = false;

  // 座標変換の判定用
  bool flag_frame_change_ = false;

  //NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  //Subscriber
  ros::Subscriber sub_target_path_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_local_map_;

  //Publisher
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_cmd_pos_;
  ros::Publisher pub_carrot1_;
  ros::Publisher pub_carrot2_;
  ros::Publisher pub_optimal_path_;

  // tf
  tf2_ros::Buffer tf_buffer_;

  // 各種オブジェクト
  nav_msgs::Path target_path_;              // 目標軌道
  nav_msgs::Odometry robot_odom_;       // ロボットの位置情報
  geometry_msgs::Pose2D current_pose_;  // ロボットの位置情報（2D）
  nav_msgs::OccupancyGrid local_map_;   // local_map
  geometry_msgs::PointStamped carrot1_;  // cmd_vel計算用ターゲット
  geometry_msgs::PointStamped carrot2_;  // ステア角計算用ターゲット
  geometry_msgs::Twist cmd_vel_;         // 制御入力（並進速度・旋回速度）
  ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;  // 制御入力（ステア角）
  nav_msgs::Path cx_line_;
};

#endif  // DUAL_PURE_PURSUIT_PLANNER_H 