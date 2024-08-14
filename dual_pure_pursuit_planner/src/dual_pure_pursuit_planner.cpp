#include "dual_pure_pursuit_planner/dual_pure_pursuit_planner.h"

DualPurePursuitPlanner::DualPurePursuitPlanner():private_nh_("~")
{
  //param
  private_nh_.param("hz", hz_, {30});
  private_nh_.param("dist_to_carrot1", dist_to_carrot1_, {1.0});
  private_nh_.param("dist_to_carrot2", dist_to_carrot2_, {0.5});
  private_nh_.param("max_target_velocity", max_target_velocity_, {1.2});  // CCV's maximum speed is 3.0 [m/s]
  private_nh_.param("max_yawrate", max_yawrate_, {4.5});                  // CCV's maximum yawrate is 4.5 [rad/s]
  private_nh_.param("max_steer_angle", max_steer_angle_, {20.0});         // Specification is 20 [deg]
  max_steer_angle_ *= M_PI/180;                                           // Convert to radian
  private_nh_.param("pitch_offset", pitch_offset_, {3.0*M_PI/180.0});
  private_nh_.param("tread", tread_, {0.48});                             // Specification is 0.5 [m]
  private_nh_.param("goal_tolerance", goal_tolerance_, {0.3});
  private_nh_.param("vel_reso", vel_reso_, {0.1});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("predict_time", predict_time_, {1.0});
  private_nh_.param("margin", margin_, {0.5});
  private_nh_.param("world_frame_id", world_frame_id_, {"odom"});
  private_nh_.param("robot_frame_id", robot_frame_id_, {"base_link"});

  //subscriber
  sub_predicted_path_ = nh_.subscribe("/predicted_path", 10, &DualPurePursuitPlanner::predicted_trajectory_callback, this, ros::TransportHints().tcpNoDelay());
  sub_robot_odom_ = nh_.subscribe("/robot_odom", 1, &DualPurePursuitPlanner::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_people_states_ = nh_.subscribe("/transformed_people_states", 1, &DualPurePursuitPlanner::people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  //publisher
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
  pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

  //debug
  pub_carrot1_ = nh_.advertise<geometry_msgs::PoseStamped>("/carrot1", 1);
  pub_carrot2_ = nh_.advertise<geometry_msgs::PoseStamped>("/carrot2", 1);
  pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
}

// target_pathのコールバック関数
void DualPurePursuitPlanner::predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg)
{
  predicted_path_=*msg;
  flag_predicted_path_ = true;

  geometry_msgs::TransformStamped tf;

  try
  {
    tf = tf_buffer_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0));

    // 目標軌跡をロボット座標系に変換
    for(auto &pose : predicted_path_.poses)
    {
      pose.pose.position.x -= tf.transform.translation.x;
      pose.pose.position.y -= tf.transform.translation.y;
    }

    // 現在のロボットの姿勢を取得
    current_pose_.x = tf.transform.translation.x;
    current_pose_.y = tf.transform.translation.y;
    current_pose_.theta = std::asin(tf.transform.rotation.z)*2.0;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}

// ロボットのodomのコールバック関数
void DualPurePursuitPlanner::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_odom_ = *msg;
  flag_robot_odom_ = true;
}

// 歩行者情報のコールバック関数
void DualPurePursuitPlanner::people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
  while(people_states_.size() > 0)
  {
    // people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    people_states_.pop();
  }

  people_states_.emplace(msg);
  flag_people_states_ = true;
}

// 距離を計算
double DualPurePursuitPlanner::calc_dist(const double &x1, const double &y1, const double &x2, const double &y2)
{
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

// ターゲットのキャロットを更新
void DualPurePursuitPlanner::update_carrots()
{
  double np = std::numeric_limits<double>::max();
  auto np_it = predicted_path_.poses.end();

  std::vector<double> distances;
  for(auto p=predicted_path_.poses.begin(); p!=predicted_path_.poses.end(); ++p)
  {
    distances.push_back(calc_dist(p->pose.position.x, p->pose.position.y, 0.0, 0.0));
  }

  double npr = np;
  auto npr_it = predicted_path_.poses.end();
  double r_max = 1.5, r_resolution = 0.1;

  for(double r=r_max; r>0; r-=r_resolution)
  {
    for(auto p=predicted_path_.poses.begin(); p!=predicted_path_.poses.end(); ++p)
    {
      double distance = calc_dist(r*sin(-current_pose_.theta), r*cos(-current_pose_.theta), p->pose.position.x, p->pose.position.y);
      if(distance<npr)
      {
        npr = distance;
        npr_it = p;
      }
      distance = calc_dist(-r*sin(-current_pose_.theta), -r*cos(-current_pose_.theta), p->pose.position.x, p->pose.position.y);
      if(distance<npr)
      {
        npr = distance;
        npr_it = p;
      }
    }
  }
  if(np_it != npr_it)
  {
    np = npr;
    np_it = npr_it;
  }

  double np1 = std::numeric_limits<double>::max();
  double np2 = std::numeric_limits<double>::max();

  for(auto p=np_it; p!=predicted_path_.poses.end(); ++p)
  {
    double distance = fabs(distances[p-predicted_path_.poses.begin()] - dist_to_carrot1_);
    if(distance < np1)
    {
      carrot1_ = p->pose;
      np1 = distance;
      carrot_distance_ = distances[p-predicted_path_.poses.begin()];
    }
    distance = fabs(distances[p-predicted_path_.poses.begin()]-dist_to_carrot2_);
    if(distance < np2)
    {
      carrot2_ = p->pose;
      np2 = distance;
    }
  }

  // キャロットをworld座標系に変換
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0));

    geometry_msgs::PoseStamped carrot1_pose;
    carrot1_pose.header.frame_id = world_frame_id_;
    carrot1_pose.pose = carrot1_;
    carrot1_pose.pose.position.x += tf.transform.translation.x;
    carrot1_pose.pose.position.y += tf.transform.translation.y;
    pub_carrot1_.publish(carrot1_pose);

    geometry_msgs::PoseStamped carrot2_pose;
    carrot2_pose.header.frame_id = world_frame_id_;
    carrot2_pose.pose = carrot2_;
    carrot2_pose.pose.position.x += tf.transform.translation.x;
    carrot2_pose.pose.position.y += tf.transform.translation.y;
    pub_carrot2_.publish(carrot2_pose);
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}

// goalに着くまでfalseを返す
bool DualPurePursuitPlanner::is_goal()
{
  const double dist = calc_dist(predicted_path_.poses.back().pose.position.x, predicted_path_.poses.back().pose.position.y, 0.0, 0.0);

  if(dist > goal_tolerance_)
    return false;
  else
    return true;
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

// obsの評価関数を計算（障害物に衝突するか）
double DualPurePursuitPlanner::calc_obs_score(const std::vector<State>& trajectory)
{
  const auto people = people_states_.front();

  // 軌跡上に障害物がないか探索
  for(const auto& state : trajectory)
  {
    for(const auto& person : people->people_states)
    {
      // pathのうちの１点と障害物の距離を計算
      const double dist = calc_dist(person.pose.position.x, person.pose.position.y, state.x, state.y);
      
      // 障害物に衝突したパスを評価
      if(dist <= margin_)
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
double DualPurePursuitPlanner::calc_steering_angle(double r, double delta, double a, char side)
{
  if(side == 'i') return std::atan2(r*sin(delta), r*cos(delta)-a);
  if(side == 'o') return std::atan2(r*sin(delta), r*cos(delta)+a);
  return 0;
}

// ステア角に基づいて速度を調整
void DualPurePursuitPlanner::flip_velocity(double &delta, double &velocity)
{
  if(delta>0.0) delta -= M_PI;
  else delta += M_PI;
  velocity *= -1;
}

// 制御入力を計算
void DualPurePursuitPlanner::update_motion()
{
  // carrotに対する方位誤差を計算
  double alpha = std::atan2(carrot1_.position.y, carrot1_.position.x) - current_pose_.theta;
  double delta = std::atan2(carrot2_.position.y, carrot2_.position.x) - current_pose_.theta;
  delta = std::max(-max_steer_angle_, std::min(max_steer_angle_, delta));

  const double a = tread_ / 2.0;  // トレッドの半分の長さ
  const double l1 = calc_dist(carrot1_.position.x, carrot1_.position.y , 0.0, 0.0);
  double r = fabs(l1 / alpha);

  double v = 0.0;  // 並進速度
  double w = 0.0;  // 旋回速度
  std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
  double max_score = -1e6;                      // 評価値の最大値格納用
  int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用
  
  // 並進速度と旋回速度のすべての組み合わせを評価
  int i = 0; // 現在の軌跡のインデックス保持用
  for(double velocity=0.0; velocity<=max_target_velocity_; velocity+=vel_reso_)
  {
    // 旋回速度を計算
    double yawrate = velocity * alpha / l1;

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
      v  = velocity;
      w  = yawrate;
      index_of_max_score = i;
    }

    i++;
  }

  // パスを可視化して適切なパスが選択できているかを評価
  for(i=0; i<trajectories.size(); i++)
  {
    if(i == index_of_max_score)
    visualize_trajectory(trajectories[i]);
  }

  double v_r = v + a*w;
  double v_l = v - a*w;
  double d_r,d_l;

  double eps = 0.1/180*M_PI;
  if(fabs(alpha)<eps)
  {
    std::cout<<"alpha is small"<<std::endl;
    d_l = delta;
    d_r = delta;
  }
  else
  {
    double d_i = calc_steering_angle(r, delta, a, 'i');
    double d_o = calc_steering_angle(r, delta, a, 'o');
    double v_i = 1.0, v_o = 1.0;
    if(fabs(d_i) > M_PI/2.0) flip_velocity(d_i,v_i);
    if(fabs(d_o) > M_PI/2.0) flip_velocity(d_o,v_o);

    if(w>0.0)
    {
      d_l = d_i;
      d_r = d_o;
      v_l *= v_i;
      v_r *= v_o;
      if(d_l>max_steer_angle_)
      {
        d_l = max_steer_angle_;
        d_r = asin((v*sin(delta)-v_l*sin(d_l))/v_r);
      }
      if(d_l<-max_steer_angle_)
      {
        d_l = -max_steer_angle_;
        d_r = asin((v*sin(delta)-v_l*sin(d_l))/v_r);
      }
    }
    else
    {
      d_l = d_o;
      d_r = d_i;
      v_l *= v_o;
      v_r *= v_i;
      if(d_r>max_steer_angle_)
      {
        d_r = max_steer_angle_;
        d_l = asin((v*sin(delta)-v_r*sin(d_r))/v_l);
      }
      if(d_r<-max_steer_angle_)
      {
        d_r = -max_steer_angle_;
        d_l = asin((v*sin(delta)-v_r*sin(d_r))/v_l);
      }
    }

  }

  geometry_msgs::Twist vel;
  vel.linear.x = (v_r + v_l)/2.0;
  vel.angular.z = (v_r - v_l)/(2.0*a);
  if(is_goal())
  {
    std::cout<<"reached goal"<<std::endl;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    d_r=0.0;
    d_l=0.0;
  }

  pub_cmd_vel_.publish(vel);
  // std::cout<<"carrot_distance:"<<carrot_distance_<<std::endl;

  ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
  cmd_pos.steer_r= d_r;
  cmd_pos.steer_l= d_l;
  cmd_pos.fore=pitch_offset_;
  cmd_pos.rear=pitch_offset_;
  cmd_pos.roll=0.0;
  pub_cmd_pos_.publish(cmd_pos);
  // printf("carrot1: %.2lf,%.2lf carrot2: %.2lf,%.2lf\n", carrot1_.position.x, carrot1_.position.y, carrot2_.position.x, carrot2_.position.y);
  std::cout<<"v: "<<(double)vel.linear.x<<" w: "<<(double)vel.angular.z<<std::endl;
  std::cout<<"alpha: "<<alpha/M_PI*180<<" delta: "<<delta/M_PI*180<<std::endl;
  // std::cout<<"r: "<<r<<" r_l: "<<r_l<<" r_r: "<<r_r<<std::endl;
  std::cout<<"steer_l: "<<-cmd_pos.steer_l/M_PI*180<<" steer_r: "<<-cmd_pos.steer_r/M_PI*180<<std::endl;
  std::cout<<"current_theta: "<<current_pose_.theta<<std::endl;
}

// 軌跡の可視化
void DualPurePursuitPlanner::visualize_trajectory(std::vector<State>& trajectory)
{
  ros::Time now = ros::Time::now();

  nav_msgs::Path local_path;
  local_path.header.stamp = now;
  local_path.header.frame_id = robot_frame_id_;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = robot_frame_id_;

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
    if((flag_predicted_path_) && (flag_robot_odom_) && (flag_people_states_))
    {
      // キャロットを更新
      update_carrots();
      
      // 制御入力を更新
      update_motion();
    }

    else ROS_WARN_STREAM("cannot receive path");
    ros::spinOnce();
    std::cout<<"=============================="<<std::endl;
    loop_rate.sleep();
  }
}