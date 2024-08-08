#include "dual_pure_pursuit_planner/dual_pure_pursuit_planner.h"

DualPurePursuitPlanner::DualPurePursuitPlanner():private_nh_("~"),tf_listener_(tf_buffer_)
{
    //param
    private_nh_.param("hz", hz_, {30});
    private_nh_.param("L1", L1_, {2.0});
    private_nh_.param("L2", L2_, {0.5});
    private_nh_.param("max_target_velocity", MAX_TARGET_VELOCITY_, {1.2});  // CCV's maximum speed is 3.0[m/s]
    private_nh_.param("max_steering_angle", MAX_STEERING_ANGLE_, {20});     // Specification is 20[deg]
    MAX_STEERING_ANGLE_ *= M_PI/180;
    private_nh_.param("pitch_offset", PITCH_OFFSET_, {3.0*M_PI/180.0});
    private_nh_.param("tread", TREAD_, {0.48});                             // Specification is 0.5[m]
    private_nh_.param("goal_border", GOAL_BORDER_, {0.3});
    private_nh_.param("read_marker", read_marker_, {true});
    private_nh_.param("world_frame_id", world_frame_id_, {"odom"});
    private_nh_.param("robot_frame_id", robot_frame_id_, {"base_link"});
    target_velocity_ = 0.0;


    //subscriber
    if(read_marker_)
        sub_marker_ = nh_.subscribe("/local_planner/state_lattice_planner/selected_trajectory", 10, &DualPurePursuitPlanner::trajectory_marker_callback, this, ros::TransportHints().tcpNoDelay());
    else
        sub_path_=nh_.subscribe("/predicted_path", 10, &DualPurePursuitPlanner::predicted_trajectory_callback, this, ros::TransportHints().tcpNoDelay());

    //publisher
    pub_cmd_vel_=nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    pub_cmd_pos_=nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

    //debug
    pub_carrot1_=nh_.advertise<geometry_msgs::PoseStamped>("/carrot1", 1);
    pub_carrot2_=nh_.advertise<geometry_msgs::PoseStamped>("/carrot2", 1);
    pub_cx_line_=nh_.advertise<nav_msgs::Path>("/cx_line",1);
}

DualPurePursuitPlanner::~DualPurePursuitPlanner(){}

void DualPurePursuitPlanner::predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg)
{
    if(read_marker_) return;
    have_received_path_=true;
    predicted_path_=*msg;

    geometry_msgs::TransformStamped tf;

    //Change the coordinate system of predicted_trajectory to odom
    try
    {
        tf = tf_buffer_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0));
        for(auto &pose : predicted_path_.poses)
        {
            pose.pose.position.x -= tf.transform.translation.x;
            pose.pose.position.y -= tf.transform.translation.y;
        }
        current_pose_.x = tf.transform.translation.x;
        current_pose_.y = tf.transform.translation.y;
        current_pose_.theta = std::asin(tf.transform.rotation.z)*2.0;
    }
    catch(tf2::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
}

void DualPurePursuitPlanner::trajectory_marker_callback(const visualization_msgs::Marker::ConstPtr &msg)
{
    if(!msg->points.size()) return;
    if(!read_marker_) return;
    nav_msgs::Path path;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = msg->header.frame_id;
    for(const auto p: msg->points)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation.w = 0.1;
        path.poses.push_back(pose);
    }
    path.header = header;
    predicted_path_ = path;
    have_received_path_ = true;
}

double DualPurePursuitPlanner::calc_distance(const double &x1, const double &y1, const double &x2, const double &y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void DualPurePursuitPlanner::update_carrots()
{
    double np=std::numeric_limits<double>::max();
    auto np_it=predicted_path_.poses.end();

    std::vector<double> distances;
    for(auto p=predicted_path_.poses.begin(); p!=predicted_path_.poses.end(); ++p)
    {
        distances.push_back(calc_distance(p->pose.position.x, p->pose.position.y, 0.0, 0.0));
        if(distances.back()<np)
        {
            // np=distances.back();
            // np_it=p;
        }
    }

    double npr=np;
    auto npr_it=predicted_path_.poses.end();
    double r_max=1.5, r_resolution=0.1;
    cx_line_.poses.clear();
    for(double r=r_max; r>0; r-=r_resolution)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id=world_frame_id_;
        pose.pose.position.x = -r*sin(current_pose_.theta);
        pose.pose.position.y = r*cos(current_pose_.theta);
        cx_line_.poses.push_back(pose);
        pose.pose.position.x *= -1;
        pose.pose.position.y *= -1;
        cx_line_.poses.insert(cx_line_.poses.begin(),pose);

        for(auto p=predicted_path_.poses.begin(); p!=predicted_path_.poses.end(); ++p)
        {
            double distance = calc_distance(r*sin(-current_pose_.theta), r*cos(-current_pose_.theta), p->pose.position.x, p->pose.position.y);
            if(distance<npr)
            {
                npr = distance;
                npr_it=p;
            }
            distance = calc_distance(-r*sin(-current_pose_.theta), -r*cos(-current_pose_.theta), p->pose.position.x, p->pose.position.y);
            if(distance<npr)
            {
                npr = distance;
                npr_it=p;
            }
        }
    }
    if(np_it != npr_it)
    {
        np = npr;
        np_it = npr_it;
    }
    geometry_msgs::Pose cx = np_it->pose;




    double np1=std::numeric_limits<double>::max();
    double np2=std::numeric_limits<double>::max();

    for(auto p=np_it; p!=predicted_path_.poses.end(); ++p)
    {
        double distance=fabs(distances[p-predicted_path_.poses.begin()]-L1_);
        if(distance<np1)
        {
            carrot1_=p->pose;
            np1=distance;
            carrot_distance_=distances[p-predicted_path_.poses.begin()];
        }
        distance=fabs(distances[p-predicted_path_.poses.begin()]-L2_);
        if(distance<np2)
        {
            carrot2_=p->pose;
            np2=distance;
        }
    }

    if(calc_distance(0.0, 0.0, predicted_path_.poses.back().pose.position.x, predicted_path_.poses.back().pose.position.y)<GOAL_BORDER_) have_reached_goal_=true;
    else have_reached_goal_=false;

    //debug
    // std::cout<<"L1_:"<<L1_<<" np1_idx:"<<np1_idx<<std::endl;
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = tf_buffer_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0));
        geometry_msgs::PoseStamped carrot1_pose;
        carrot1_pose.header.frame_id=world_frame_id_;
        carrot1_pose.pose=carrot1_;
        carrot1_pose.pose.position.x += tf.transform.translation.x;
        carrot1_pose.pose.position.y += tf.transform.translation.y;
        pub_carrot1_.publish(carrot1_pose);
        geometry_msgs::PoseStamped carrot2_pose;
        carrot2_pose.header.frame_id=world_frame_id_;
        carrot2_pose.pose=carrot2_;
        // carrot2_pose.pose = cx;
        // carrot2_pose.pose.position.x = current_pose_.x;
        // carrot2_pose.pose.position.y = current_pose_.y;
        // carrot2_pose.pose.orientation.z = sin(current_pose_.theta/2.0);
        // carrot2_pose.pose.orientation.w = cos(current_pose_.theta/2.0);
        carrot2_pose.pose.position.x += tf.transform.translation.x;
        carrot2_pose.pose.position.y += tf.transform.translation.y;
        pub_carrot2_.publish(carrot2_pose);
        cx_line_.header.frame_id = world_frame_id_;
        for(auto &p : cx_line_.poses)
        {
            p.pose.position.x += tf.transform.translation.x;
            p.pose.position.y += tf.transform.translation.y;
        }
        pub_cx_line_.publish(cx_line_);

    }
    catch(tf2::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

}


double DualPurePursuitPlanner::calc_steering_angle(double r, double delta, double a, char side)
{
    if(side == 'i') return std::atan2(r*sin(delta), r*cos(delta)-a);
    if(side == 'o') return std::atan2(r*sin(delta), r*cos(delta)+a);
    return 0;
}

double DualPurePursuitPlanner::calc_turn_radius(double r,double delta, double a, char side)
{
    if(side == 'i') return sqrt(a*a + 2*a*r*sin(delta) + r*r);
    if(side == 'o') return sqrt(a*a - 2*a*r*sin(delta) + r*r);
    return 0;
}

void DualPurePursuitPlanner::flip_velocity(double &delta, double &velocity)
{
    if(delta>0.0) delta -= M_PI;
    else delta += M_PI;
    velocity *= -1;
}

void DualPurePursuitPlanner::update_motion()
{
    // double v=target_velocity_;
    double v=MAX_TARGET_VELOCITY_;
    double alpha = std::atan2(carrot1_.position.y, carrot1_.position.x) - current_pose_.theta;
    double delta = std::atan2(carrot2_.position.y, carrot2_.position.x) - current_pose_.theta;
    double carrot_distance=calc_distance(carrot1_.position.x, carrot1_.position.y , 0.0, 0.0);
    double w = v*alpha/carrot_distance;
    double r = fabs(carrot_distance/alpha);
    double a = TREAD_/2.0;
    delta = std::max(-MAX_STEERING_ANGLE_, std::min(MAX_STEERING_ANGLE_, delta));
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
            if(d_l>MAX_STEERING_ANGLE_)
            {
                d_l = MAX_STEERING_ANGLE_;
                // d_r = 0.0;
                d_r = asin((v*sin(delta)-v_l*sin(d_l))/v_r);
            }
            if(d_l<-MAX_STEERING_ANGLE_)
            {
                d_l = -MAX_STEERING_ANGLE_;
                // d_r = 0.0;
                d_r = asin((v*sin(delta)-v_l*sin(d_l))/v_r);
            }
        }
        else
        {
            d_l = d_o;
            d_r = d_i;
            v_l *= v_o;
            v_r *= v_i;
            if(d_r>MAX_STEERING_ANGLE_)
            {
                d_r = MAX_STEERING_ANGLE_;
                d_l = asin((v*sin(delta)-v_r*sin(d_r))/v_l);
                // d_l = 0.0;
            }
            if(d_r<-MAX_STEERING_ANGLE_)
            {
                d_r = -MAX_STEERING_ANGLE_;
                d_l = asin((v*sin(delta)-v_r*sin(d_r))/v_l);
                // d_l = 0.0;
            }
        }

    }

    geometry_msgs::Twist vel;
    vel.linear.x = (v_r + v_l)/2.0;
    vel.angular.z = (v_r - v_l)/(2.0*a);
    if(have_reached_goal_)
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
    cmd_pos.fore=PITCH_OFFSET_;
    cmd_pos.rear=PITCH_OFFSET_;
    cmd_pos.roll=0.0;
    pub_cmd_pos_.publish(cmd_pos);
    // printf("carrot1: %.2lf,%.2lf carrot2: %.2lf,%.2lf\n", carrot1_.position.x, carrot1_.position.y, carrot2_.position.x, carrot2_.position.y);
    std::cout<<"v: "<<(double)vel.linear.x<<" w: "<<(double)vel.angular.z<<std::endl;
    std::cout<<"alpha: "<<alpha/M_PI*180<<" delta: "<<delta/M_PI*180<<std::endl;
    // std::cout<<"rcos(delta)-a: "<<r*cos(delta)-a<<std::endl;
    // std::cout<<"r: "<<r<<" r_l: "<<r_l<<" r_r: "<<r_r<<std::endl;
    std::cout<<"steer_l: "<<-cmd_pos.steer_l/M_PI*180<<" steer_r: "<<-cmd_pos.steer_r/M_PI*180<<std::endl;
    std::cout<<"current_theta: "<<current_pose_.theta<<std::endl;
}

#if 0
void DualPurePursuitPlanner::update_target_velocity()
{
    if(nearest_scan_.distance<SAFTY_RADIUS_)
    {
        target_velocity_ = 0.0;
        return ;
    }
    double max_distance=3.0;
    target_velocity_ = nearest_scan_.distance/max_distance*MAX_TARGET_VELOCITY_;
    if(target_velocity_>max_distance) target_velocity_ = MAX_TARGET_VELOCITY_;

}
#endif

void DualPurePursuitPlanner::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(have_received_path_)
        {
            update_carrots();
            // update_target_velocity();
            update_motion();
        }
        else ROS_WARN_STREAM("cannot receive path");
        ros::spinOnce();
        std::cout<<"=============================="<<std::endl;
        loop_rate.sleep();
    }

}