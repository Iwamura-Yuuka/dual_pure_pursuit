#ifndef DUAL_PURE_PURSUIT_PLANNER_H
#define DUAL_PURE_PURSUIT_PLANNER_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>

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
  // コールバック関数
  void predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg);
  void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);

  // 引数なし関数
  bool is_goal();                                                                           // goalに着くまでfalseを返す
  void update_carrots();
  void update_motion();

  // 引数あり関数
  double calc_dist(const double &x1, const double &y1, const double &x2, const double &y2);             // 距離を計算
  double normalize_angle(double theta);                                                                 // 適切な角度(-M_PI~M_PI)に変換
  void move_image(State& imstate, const double velocity, const double yawrate, const double delta);     // 仮想ロボットを移動
  std::vector<State> calc_trajectory(const double velocity, const double yawrate, const double delta);  // 予測軌跡を作成
  double calc_evaluation(const std::vector<State>& trajectory);                                         // 評価関数を計算
  double calc_obs_score(const std::vector<State>& trajectory);                                          // obsの評価関数を計算（障害物に衝突するか）
  double calc_vel_score(const std::vector<State>& trajectory);                                          // 並進速度の評価関数を計算
  double calc_steering_angle(double r, double delta, double a, char side);                              // ステア角を計算
  void flip_velocity(double &delta, double &velocity);                                                  // ステア角に基づいて速度を調整
  void visualize_trajectory(std::vector<State>& trajectory);                                            // 軌跡を可視化


  //yamlファイルで設定可能な変数
  int hz_;                      // ループ周波数 [Hz]
  double dist_to_carrot1_;      // carrot1（cmd_vel計算用ターゲット）までの距離 [m]
  double dist_to_carrot2_;      // carrot2（ステア角計算用ターゲット）までの距離 [m]
  double max_target_velocity_;  // 目標並進速度 [m/s]
  double max_yawrate_;          // 最高旋回速度 [rad/s]
  double max_steer_angle_;      // ステア角の最大値 [deg]
  double tread_;                // CCVのトレッド幅 [m]
  double pitch_offset_;         // offset
  double goal_tolerance_;       // ゴールに対する許容誤差 [m]
  double vel_reso_;             // 並進速度を計算するときの刻み幅 [m/s]
  double dt_;                   // 微小時間 [s]
  double predict_time_;         // 予測時間 [s]
  double margin_;               // 障害物とロボットの衝突半径（マージン込み） [m]

  // msgの受け取り判定用
  bool flag_predicted_path_ = false;
  bool flag_robot_odom_ = false;
  bool flag_people_states_ = false;

  // 座標変換の判定用
  bool flag_frame_change_ = false;
  


  //member
  geometry_msgs::Pose carrot1_, carrot2_;
  double carrot_distance_;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;


  // Subscriber
  ros::Subscriber sub_predicted_path_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_people_states_;
  

  // Publisher
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
  ros::Publisher pub_optimal_path_;

  // 各種オブジェクト
  nav_msgs::Path predicted_path_;         // 目標軌跡
  nav_msgs::Odometry robot_odom_;         // ロボットの位置情報
  std::queue<pedestrian_msgs::PeopleStatesConstPtr> people_states_;  // 歩行者情報


};

#endif  // DUAL_PURE_PURSUIT_PLANNER_H 