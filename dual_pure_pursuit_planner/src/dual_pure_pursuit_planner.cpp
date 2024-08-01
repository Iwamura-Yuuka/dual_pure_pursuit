#include "dual_pure_pursuit_planner/dual_pure_pursuit_planner.h"

DualPurePursuitPlanner::DualPurePursuitPlanner():private_nh_("~")
{
  // param
  private_nh_.param("visualize_for_debug", visualize_for_debug_, {false});
  private_nh_.param("use_global_path", use_global_path_, {false});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("dist_to_carrot1", dist_to_carrot1_, {1.0});
  private_nh_.param("dist_to_carrot2", dist_to_carrot2_, {0.5});
  private_nh_.param("world_frame", world_frame_, {"odom"});
  private_nh_.param("robot_frame", robot_frame_, {"base_footprint"});
  private_nh_.param("tmp_carrot1_index", tmp_carrot1_index_, {0});
  private_nh_.param("tmp_carrot2_index", tmp_carrot2_index_, {0});

  // subscriber
  sub_target_path_ = nh_.subscribe("/predicted_path", 1, &DualPurePursuitPlanner::target_path_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_robot_odom_ = nh_.subscribe("/robot_odom", 1, &DualPurePursuitPlanner::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  // pub_predicted_states_ = nh_.advertise<std_msgs::Float32MultiArray>("/predicted_state", 1);

  // debug
  if(visualize_for_debug_)
  {
    pub_carrot1_ = nh_.advertise<geometry_msgs::PointStamped>("/carrot11", 1);
    pub_carrot2_ = nh_.advertise<geometry_msgs::PointStamped>("/carrot22", 1);
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
void DualPurePursuitPlanner::update_carrots()
{
  ros::Time now = ros::Time::now();

  carrot1_.header.stamp = now;
  carrot1_.header.frame_id = target_path_.header.frame_id;
  carrot2_.header.stamp = now;
  carrot2_.header.frame_id = target_path_.header.frame_id;

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

// 制御入力を計算
void DualPurePursuitPlanner::update_motion()
{
  // 制御入力を計算
  // ここに処理を記述
}










// メイン文で実行する関数
void DualPurePursuitPlanner::process()
{
  ros::Rate loop_rate(hz_);
  tf2_ros::TransformListener tf_listener(tf_buffer_);

  while(ros::ok())
  {
    if((flag_target_path_) && (flag_robot_odom_))
    {
      ROS_INFO_STREAM("receive path");
      // ターゲットのキャロットを更新
      update_carrots();

      // 制御入力を計算
      update_motion();
    }
    else
    {
      ROS_WARN_STREAM("cannot receive path");
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_target_path_ = false;
    flag_robot_odom_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}