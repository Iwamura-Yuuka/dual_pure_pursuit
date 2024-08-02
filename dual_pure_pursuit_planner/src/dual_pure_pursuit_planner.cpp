#include "dual_pure_pursuit_planner/dual_pure_pursuit_planner.h"

DualPurePursuitPlanner::DualPurePursuitPlanner():private_nh_("~")
{
  // param
  private_nh_.param("visualize_for_debug", visualize_for_debug_, {false});
  private_nh_.param("use_global_path", use_global_path_, {false});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("max_target_velocity", max_target_velocity_, {1.2});  // CCV's maximum speed is 3.0[m/s]
  private_nh_.param("max_steer_angle", max_steer_angle_, {20.0});         // Specification is 20[deg]
  max_steer_angle_ *= M_PI / 180.0;                                       // Convert to [rad]
  private_nh_.param("tread", tread_, {0.5});                              // Specification is 0.5[m]
  private_nh_.param("dist_to_carrot1", dist_to_carrot1_, {1.0});
  private_nh_.param("dist_to_carrot2", dist_to_carrot2_, {0.5});
  private_nh_.param("goal_tolerance", goal_tolerance_, {0.5});
  private_nh_.param("world_frame", world_frame_, {"odom"});
  private_nh_.param("robot_frame", robot_frame_, {"base_footprint"});
  private_nh_.param("tmp_carrot1_index", tmp_carrot1_index_, {0});
  private_nh_.param("tmp_carrot2_index", tmp_carrot2_index_, {0});
  private_nh_.param("max_yawrate", max_yawrate_, {4.5});
  private_nh_.param("vel_reso", vel_reso_, {0.1});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("predict_time", predict_time_, {1.0});

  // subscriber
  sub_target_path_ = nh_.subscribe("/predicted_path", 10, &DualPurePursuitPlanner::target_path_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_robot_odom_ = nh_.subscribe("/robot_odom", 1, &DualPurePursuitPlanner::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_local_map_ = nh_.subscribe("/local_map", 10, &DualPurePursuitPlanner::local_map_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_cmd_vel_=nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
  pub_cmd_pos_=nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

  // debug
  if(visualize_for_debug_)
  {
    pub_carrot1_ = nh_.advertise<geometry_msgs::PointStamped>("/carrot11", 1);
    pub_carrot2_ = nh_.advertise<geometry_msgs::PointStamped>("/carrot22", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
  }
}

// target_pathのコールバック関数
void DualPurePursuitPlanner::target_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
  target_path_ = *msg;
  flag_target_path_ = true;
}

// ロボットのodomのコールバック関数
void DualPurePursuitPlanner::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_odom_ = *msg;
  flag_robot_odom_ = true;
}

// local_mapのコールバック関数
void DualPurePursuitPlanner::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  local_map_ = *msg;
  flag_local_map_ = true;
}

// 距離を計算
double DualPurePursuitPlanner::calc_dist(const double x1, const double y1, const double x2, const double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;

  return hypot(dx, dy);
}

// ターゲットのキャロットの位置を計算
void DualPurePursuitPlanner::calc_carrot(geometry_msgs::PointStamped& carrot, const double dist_to_carrot)
{
  int target_path_index = 0;  // target_pathのインデックス

  // global_pathをtarget_pathとするときは，tmp_carrot_indexを使ってtarget_path_indexを初期化
  if(use_global_path_)
  {
    if(dist_to_carrot == dist_to_carrot1_)  // carrot1
    {
      target_path_index = tmp_carrot1_index_;
    }
    else if(dist_to_carrot == dist_to_carrot2_)  // carrot2
    {
      target_path_index = tmp_carrot2_index_;
    }
  }

  // 距離を計算
  double dist = calc_dist(target_path_.poses[target_path_index].pose.position.x, target_path_.poses[target_path_index].pose.position.y, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);
  double min_error = std::fabs(dist - dist_to_carrot);

  while(target_path_index < target_path_.poses.size())
  {
    // パスのインデックスを進める
    target_path_index++;

    // 距離を計算
    dist = calc_dist(target_path_.poses[target_path_index].pose.position.x, target_path_.poses[target_path_index].pose.position.y, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);
    const double error = std::fabs(dist - dist_to_carrot);

    // 最小誤差を更新
    if(error < min_error)
    {
      min_error = error;
    }
    else
    {
      // 1つ前のインデックスをキャロットとして返す
      target_path_index--;
      break;
    }

    // target_path_indexがtarget_path_.posesの配列の最後に到達した場合，carrotとしてtarget_path_.posesの最後の要素を返す
    if(target_path_index == target_path_.poses.size()-1)
    {
      break;
    }
  }

  // carrotの更新
  carrot.point.x = target_path_.poses[target_path_index].pose.position.x;
  carrot.point.y = target_path_.poses[target_path_index].pose.position.y;
  carrot.point.z = 0.0;

  // tmp_carrot_indexの更新
  if(dist_to_carrot == dist_to_carrot1_)  // carrot1
  {
    tmp_carrot1_index_ = target_path_index;
  }
  else if(dist_to_carrot == dist_to_carrot2_)  // carrot2
  {
    tmp_carrot2_index_ = target_path_index;
  }
}

// キャロットをodom座標系からbase_footprint座標系に変換
void DualPurePursuitPlanner::transform_carrot(geometry_msgs::PointStamped& carrot)
{
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform(robot_frame_, world_frame_, ros::Time(0));
    flag_frame_change_ = true;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    flag_frame_change_ = false;
    return;
  }

  geometry_msgs::PointStamped after_carrot;
  tf2::doTransform(carrot, after_carrot, transform);

  carrot = after_carrot;
}

// ターゲットのキャロットを更新
void DualPurePursuitPlanner::update_carrots(ros::Time now)
{
  carrot1_.header.stamp = now;
  carrot1_.header.frame_id = world_frame_;
  carrot2_.header.stamp = now;
  carrot2_.header.frame_id = world_frame_;

  // キャロットを更新
  calc_carrot(carrot1_, dist_to_carrot1_);  // carrot1
  calc_carrot(carrot2_, dist_to_carrot2_);  // carrot2

  // キャロットをodom座標系からbase_footprint座標系に変換
  transform_carrot(carrot1_);  // carrot1
  transform_carrot(carrot2_);  // carrot2

  // キャロットを可視化
  if((visualize_for_debug_) && (flag_frame_change_))
  {
    pub_carrot1_.publish(carrot1_);
    pub_carrot2_.publish(carrot2_);
  }
}

// goalに着くまでtrueを返す
bool DualPurePursuitPlanner::is_goal()
{
  const double dist = calc_dist(carrot1_.point.x, carrot1_.point.y, 0.0, 0.0);

  if(dist > goal_tolerance_)
    return true;
  else
    return false;
}

// 適切な角度(-M_PI~M_PI)に変換
double DualPurePursuitPlanner::normalize_angle(double theta)
{
  if(theta > M_PI)
    theta -= 2.0 * M_PI;
  if(theta < -M_PI)
    theta += 2.0 * M_PI;

  return theta;
}

// 仮想ロボットを移動
void DualPurePursuitPlanner::move_image(State& imstate, const double velocity, const double yawrate, const double delta)
{
  imstate.yaw += yawrate * dt_;
  imstate.yaw  = normalize_angle(imstate.yaw);
  imstate.x += velocity * dt_ * cos(normalize_angle(imstate.yaw + delta));
  imstate.y += velocity * dt_ * sin(normalize_angle(imstate.yaw + delta));
  imstate.velocity = velocity;
  imstate.yawrate = yawrate;
}

// 予測軌跡を作成
std::vector<State> DualPurePursuitPlanner::calc_trajectory(const double velocity, const double yawrate, const double delta)
{
  std::vector<State> traj;                    //軌跡格納用の動的配列
  State imstate = {0.0, 0.0, 0.0, 0.0, 0.0};  //仮想ロボット

  //軌跡を格納
  for(double t=0.0; t<=predict_time_; t+=dt_)
  {
    move_image(imstate, velocity, yawrate, delta);
    traj.push_back(imstate);  //trajの末尾に挿入
  }

  return traj;
}

// 評価関数を計算
double DualPurePursuitPlanner::calc_evaluation(const std::vector<State>& trajectory)
{
  const double obs_score = calc_obs_score(trajectory);
  const double vel_score = calc_vel_score(trajectory);

  const double total_score = obs_score + vel_score;

  return total_score;
}

// 座標からグリッドのインデックスを返す
int DualPurePursuitPlanner::xy_to_grid_index(const double x, const double y)
{
  const int index_x = int(floor((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
  const int index_y = int(floor((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

  return index_x + (index_y * local_map_.info.width);
}

// obsの評価関数を計算（障害物に衝突するか）
double DualPurePursuitPlanner::calc_obs_score(const std::vector<State>& trajectory)
{
  // 軌跡上に障害物がないか探索
  for(const auto& state : trajectory)
  {
    const int grid_index = xy_to_grid_index(state.x, state.y);

    // 衝突したパスにはマイナス値を返す
    if(local_map_.data[grid_index] == 100)
    {
      return -1e6;
    }
  }

  return 0.0;
}

// 並進速度の評価関数を計算
double DualPurePursuitPlanner::calc_vel_score(const std::vector<State>& trajectory)
{
  const double velocity = trajectory.back().velocity;

  return velocity / max_target_velocity_;  // 正規化
}

// ステア角を計算
double DualPurePursuitPlanner::calc_steering_angle(const double r, const double delta, const double d, const char side)
{
  double delta_inside = 0.0;
  double delta_outside = 0.0;

  if(side == 'i')
    return atan2(r*sin(delta), r*cos(delta)-d);
  else if(side == 'o')
    return atan2(r*sin(delta), r*cos(delta)+d);
  else
    return 0.0;
}

// ステア角の最大値に基づき修正
double DualPurePursuitPlanner::fix_steer_angle(const double r, const double delta_inside, const double d, int sign)
{
  double delta = 0.0;
  double min_error = 1e6;

  if(sign == 1)  // delta_inside > max_steer_angle_のとき
  {
    for(double angle=0.0; angle<=max_steer_angle_; angle+=0.01)
    {
      double d_i = calc_steering_angle(r, angle, d, 'i');
      double error = std::fabs(delta_inside - d_i);

      if(error < min_error)
      {
        min_error = error;
        delta = angle;
      }
    }
  }
  else if(sign == -1)  // delta_inside < -max_steer_angle_のとき
  {
    for(double angle=0.0; angle>=-max_steer_angle_; angle-=0.01)
    {
      double d_i = calc_steering_angle(r, angle, d, 'i');
      double error = std::fabs(delta_inside - d_i);

      if(error < min_error)
      {
        min_error = error;
        delta = angle;
      }
    }
  }

  return delta;
}

// 制御入力を計算
void DualPurePursuitPlanner::update_motion(ros::Time now)
{
  if(is_goal())
  {
    // carrotに対する方位誤差を計算
    double alpha = atan2(carrot1_.point.y, carrot1_.point.x);
    double delta = atan2(carrot2_.point.y, carrot2_.point.x);

    // ステア角の最大値に基づきdeltaを修正
    if(delta > max_steer_angle_)
    {
      delta = max_steer_angle_;
    }
    else if(delta < -max_steer_angle_)
    {
      delta = -max_steer_angle_;
    }

    const double d = tread_ / 2.0;  // トレッドの半分の長さ

    // キャロットまでの距離を計算
    const double l1 = calc_dist(carrot1_.point.x, carrot1_.point.y, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);

    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 並進速度と旋回速度のすべての組み合わせを評価
    int i = 0; // 現在の軌跡のインデックス保持用
    for(double velocity=0.0; velocity<=max_target_velocity_; velocity+=vel_reso_)
    {
      // 旋回速度を計算
      double yawrate = 2.0 * velocity * sin(alpha) / l1;

      if(yawrate > max_yawrate_)
      {
        yawrate = max_yawrate_;
      }
      else if(yawrate < -max_yawrate_)
      {
        yawrate = -max_yawrate_;
      }

      const std::vector<State> trajectory = calc_trajectory(velocity, yawrate, delta);  // 予測軌跡の作成
      const double score = calc_evaluation(trajectory);                                 // 予測軌跡に対する評価値の計算
      trajectories.push_back(trajectory);

      // 最大値の更新
      if(max_score < score)
      {
        max_score = score;
        input[0]  = velocity;
        input[1]  = yawrate;
        index_of_max_score = i;
      }

      i++;
    }

    // パスを可視化して適切なパスが選択できているかを評価
    if(visualize_for_debug_)
    {
      for(i=0; i<trajectories.size(); i++)
      {
        if(i == index_of_max_score)
          visualize_trajectory(trajectories[i], now);
      }
    }

    // 並進速度と旋回速度
    cmd_vel_.linear.x = input[0];
    cmd_vel_.angular.z = input[1];

    // 回転半径を計算
    const double r = std::fabs(input[0] / input[1]);

    // ステア角を計算（内側・外側）
    double delta_inside = calc_steering_angle(r, delta, d, 'i');
    double delta_outside = calc_steering_angle(r, delta, d, 'o');

    // ステア角の最大値に基づきdeltaを修正
    if(delta_inside > max_steer_angle_)
    {
      delta_inside = max_steer_angle_;

      // delta_insideに合わせてdeltaを再計算
      delta = fix_steer_angle(r, delta_inside, d, 1);

      // delta_outsideを再計算
      delta_outside = calc_steering_angle(r, delta, d, 'o');
    }
    else if(delta_inside < -max_steer_angle_)
    {
      delta_inside = -max_steer_angle_;

      // delta_insideに合わせてdeltaを修正
      delta = fix_steer_angle(r, delta_inside, d, -1);

      // delta_outsideを再計算
      delta_outside = calc_steering_angle(r, delta, d, 'o');
    }

    double delta_r = 0.0;
    double delta_l = 0.0;

    // ステア各を計算（左右）
    if(input[1] > 0.0)
    {
      delta_r = delta_outside;
      delta_l = delta_inside;
    }
    else
    {
      delta_r = delta_inside;
      delta_l = delta_outside;
    }

    // ステア角
    cmd_pos_.steer_r = delta_r;
    cmd_pos_.steer_l = delta_l;
  }
  else
  {
    ROS_INFO_STREAM("====== goal!! ======");
    
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    cmd_pos_.steer_r = 0.0;
    cmd_pos_.steer_l = 0.0;
  }
  
  pub_cmd_vel_.publish(cmd_vel_);
  pub_cmd_pos_.publish(cmd_pos_);
}

// 軌跡の可視化
void DualPurePursuitPlanner::visualize_trajectory(std::vector<State>& trajectory, ros::Time now)
{
  nav_msgs::Path local_path;
  local_path.header.stamp = now;
  local_path.header.frame_id = robot_frame_;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = robot_frame_;

  // 軌跡を格納
  for(auto& state : trajectory)
  {
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    local_path.poses.push_back(pose);
  }

  pub_optimal_path_.publish(local_path);
}

// メイン文で実行する関数
void DualPurePursuitPlanner::process()
{
  ros::Rate loop_rate(hz_);
  tf2_ros::TransformListener tf_listener(tf_buffer_);

  while(ros::ok())
  {
    if((flag_target_path_) && (flag_robot_odom_) && (flag_local_map_))
    {
      ros::Time now = ros::Time::now();

      ROS_INFO_STREAM("receive path");
      // ターゲットのキャロットを更新
      update_carrots(now);

      // 制御入力を計算
      update_motion(now);
    }
    else
    {
      ROS_WARN_STREAM("cannot receive path");

      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
      cmd_pos_.steer_r = 0.0;
      cmd_pos_.steer_l = 0.0;

      if(!flag_target_path_)
      {
        ROS_WARN_STREAM("cannot receive target path");
      }
      if(!flag_robot_odom_)
      {
        ROS_WARN_STREAM("cannot receive robot odom");
      }
      if(!flag_local_map_)
      {
        ROS_WARN_STREAM("cannot receive local map");
      }

      pub_cmd_vel_.publish(cmd_vel_);
      pub_cmd_pos_.publish(cmd_pos_);
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_target_path_ = false;
    flag_robot_odom_ = false;
    // flag_local_map_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}