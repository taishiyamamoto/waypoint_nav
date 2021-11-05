#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"

#include <vector>
#include <list>
#include <string>
#include <unordered_map>
#include <exception>
#include <math.h>
#include <fstream>
#include <iostream>

typedef struct Waypoints{
  geometry_msgs::Pose pose;
  std::string function;
}Waypoints;

class WaypointNav{
public:
  WaypointNav();
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  double max_update_rate_;
  bool read_yaml();
  void compute_orientation();
  void visualize_wp();
  void run_wp();
  bool on_wp();
  void send_wp();
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);
  void timerCallback(const ros::TimerEvent& e);
  bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool suspendNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  double get_min_scan_data(const sensor_msgs::LaserScan::ConstPtr& scan_msg, double degree);

// declear functions which is called by depending on "function" in yaml
  void run();
  void suspend();
  void line_tracking();

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
  std::list<Waypoints> waypoints_;
  decltype(waypoints_)::iterator current_waypoint_;
  std::string robot_frame_, world_frame_;
  std::string filename_;
  bool loop_flg_;
  bool suspend_flg_;
  double dist_err_;
  double last_moved_time_;
  double wait_time_;
  int resend_thresh_;
  std::unordered_map<std::string, std::function<void(void)>> function_map_;
  ros::Rate rate_;
  ros::ServiceServer start_server_, suspend_server_; 
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher visualization_wp_pub_;
  ros::ServiceClient clear_costmaps_srv_;
  ros::Timer timer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  bool line_tracking_flg_;
  double min_scan_data_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber scan_sub_;
};

WaypointNav::WaypointNav() :
    nh_(),
    pnh_("~"),
    move_base_action_("move_base", true),
    rate_(1.0),
    loop_flg_(false),
    suspend_flg_(true),
    line_tracking_flg_(false),
    tfListener_(tfBuffer_),
    last_moved_time_(ros::Time::now().toSec()),
    wait_time_(5.0),
    resend_thresh_(3)
{
  pnh_.param("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param("world_frame", world_frame_, std::string("map"));

  pnh_.param("max_update_rate", max_update_rate_, 1.0);
  ros::Rate rate_(max_update_rate_);

  pnh_.param("filename", filename_, filename_);
  pnh_.param("dist_err", dist_err_, 1.0);

  pnh_.param("loop_flg", loop_flg_, false);
  pnh_.param("wait_time", wait_time_, 5.0);
  pnh_.param("resend_thresh", resend_thresh_, 3);

  function_map_.insert(std::make_pair("run", std::bind(&WaypointNav::run, this)));
  function_map_.insert(std::make_pair("suspend", std::bind(&WaypointNav::suspend, this)));
  function_map_.insert(std::make_pair("line_tracking", std::bind(&WaypointNav::line_tracking, this)));

  visualization_wp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_wp", 1);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &WaypointNav::cmdVelCallback, this);
  start_server_ = nh_.advertiseService("start_wp_nav", &WaypointNav::startNavigationCallback, this);
  suspend_server_ = nh_.advertiseService("suspend_wp_nav", &WaypointNav::suspendNavigationCallback, this);
  clear_costmaps_srv_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  timer_ = nh_.createTimer(ros::Duration(0.1),&WaypointNav::timerCallback,this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("pub_cmd_vel",1);
  scan_sub_ = nh_.subscribe("scan", 1, &WaypointNav::scanCallback, this);
  line_tracking_timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointNav::linetimerCallback, this);
}

bool WaypointNav::read_yaml(){
  ROS_INFO_STREAM("Read waypoints data from " << filename_);
// Check whether filename.yaml exist
  std::ifstream ifs(filename_);
  if(ifs){
    const YAML::Node read_result = YAML::LoadFile(filename_);
    YAML::Node wp_yaml;
    try{
      wp_yaml = read_result["waypoints"];
    }
    catch(std::exception e){
      ROS_ERROR("Your yaml format is wrong");
      return false;
    }

    geometry_msgs::Pose pose;
    std::string function;
    for(YAML::Node points : wp_yaml){
      pose.position.x = points["point"]["x"].as<double>();
      pose.position.y = points["point"]["y"].as<double>();
      pose.position.z = points["point"]["z"].as<double>();

      try{
        function = points["point"]["function"].as<std::string>();
      }
      catch(std::exception e){
        ROS_WARN("function is set by default (run) because function is not set in yaml");
        function = std::string("run");
      }
      if(function == ""){
        function = "run";
      }
      waypoints_.push_back({pose, function});
      }
    ROS_INFO_STREAM(waypoints_.size() << " waypoints is read");
    return true;
  }
  else{
    ROS_ERROR("yaml filename is wrong");
    return false;
  }
}

void WaypointNav::compute_orientation(){
  decltype(waypoints_)::iterator it, it2;
  double goal_direction;
  tf2::Quaternion calc_orientation;
  for(it = waypoints_.begin(), it2 = std::next(waypoints_.begin()); it != waypoints_.end(); it++, it2++){
    if(it2 != waypoints_.end()){
      goal_direction = atan2((it2)->pose.position.y - (it)->pose.position.y,
                              (it2)->pose.position.x - (it)->pose.position.x);

      calc_orientation.setRPY(0, 0, goal_direction);
      it->pose.orientation = tf2::toMsg(calc_orientation);
    }
    else{
      // set direction which is same as previous one
      it->pose.orientation = std::prev(it)->pose.orientation;
    }
  }
}

// This function has a bug which can't visualize waypotins
void WaypointNav::visualize_wp(){
  int cnt = 0;
  int waypoint_num = waypoints_.size();
  geometry_msgs::Vector3 arrow; // config arrow shape
  // x is arrow length
  arrow.x = 1.0;
  // y is arrow width
  arrow.y = 0.1;
  // z is arrow height
  arrow.z = 0.2;

  visualization_msgs::MarkerArray marker_wp;
  marker_wp.markers.resize(waypoint_num);
  for(decltype(waypoints_)::iterator it = waypoints_.begin(); it != waypoints_.end(); cnt++, it++){
    marker_wp.markers[cnt].header.frame_id = world_frame_;
    marker_wp.markers[cnt].header.stamp = ros::Time::now();
    marker_wp.markers[cnt].ns = "visualization_waypoint";
    marker_wp.markers[cnt].id = cnt;
    marker_wp.markers[cnt].lifetime = ros::Duration();

    marker_wp.markers[cnt].type = visualization_msgs::Marker::ARROW;
    marker_wp.markers[cnt].action = visualization_msgs::Marker::ADD;
    marker_wp.markers[cnt].scale= arrow;
    marker_wp.markers[cnt].pose = it->pose;

    marker_wp.markers[cnt].color.r = 0.0f;
    marker_wp.markers[cnt].color.g = static_cast<float>(cnt) / static_cast<float>(waypoint_num);
    marker_wp.markers[cnt].color.b = 1.0f;
    marker_wp.markers[cnt].color.a = 1.0f;
  }
  //ROS_INFO("Published waypoint marker");
  visualization_wp_pub_.publish(marker_wp);
}

void WaypointNav::run_wp(){
  while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && ros::ok()){
      ROS_INFO("Waiting...");
  }

  // If loop_flg_ is true, do_while loop infinitely
  do{
    current_waypoint_ = waypoints_.begin();
    while((current_waypoint_ != waypoints_.end()) && ros::ok()){
      if(!suspend_flg_){
      // execute a function depending on "function" written in yaml
        auto func_it = function_map_.find(current_waypoint_->function);
        if (func_it != function_map_.end()){
          func_it->second();
          current_waypoint_++;
        }
        else{
          ROS_ERROR_STREAM("Function " + current_waypoint_->function + " Is Not Found.");
        }
      }
      else if(line_tracking_flg_ && move_base_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Line tracking mode start!");
        ros::Rate rate(10.0);
        double dist = 10000.0;

        //ゲインパラメータ
        double alpha = -2.0;
        double beta = -4.0;

        //Lineの始点をロボットの現在位置から取得
        geometry_msgs::TransformStamped transformStamped;
        try{
          transformStamped = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
        }

        double start_trans_x = transformStamped.transform.translation.x;
        double start_trans_y = transformStamped.transform.translation.y;
        double start_rot_x = transformStamped.transform.rotation.x;
        double start_rot_y = transformStamped.transform.rotation.y;
        double start_rot_z = transformStamped.transform.rotation.z;
        double start_rot_w = transformStamped.transform.rotation.w;

        tf2::Quaternion quat(start_rot_x,start_rot_y,start_rot_z,start_rot_w);
        double start_euler_r, start_euler_p, start_euler_y;
        tf2::Matrix3x3(quat).getRPY(start_euler_r,start_euler_p,start_euler_y);

        //Lineの終点とロボットの距離が一定値以下になるまでループ
        while(dist > 0.5){
          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0));
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
          }

          double robot_x = transformStamped.transform.translation.x;
          double robot_y = transformStamped.transform.translation.y;
          double wp_x = current_waypoint_->pose.position.x;
          double wp_y = current_waypoint_->pose.position.y;

          //Lineの終点とロボットの距離
          dist = std::hypot((wp_x - robot_x), (wp_y - robot_y));

          //ROS_INFO("dist=%lf",dist);
          
          //Lineの角度
          double line_angular = std::atan2(current_waypoint_->pose.position.y-start_trans_y, current_waypoint_->pose.position.x-start_trans_x);

          //Lineとロボットの角度の差
          double angular_diff = start_euler_y - line_angular; 

          if(angular_diff < -M_PI){
            angular_diff = angular_diff + 2*M_PI;
          }
          //Lineの始点とロボットの位置を結んだ線の角度
          double point_angular = std::atan2(robot_y-start_trans_y, robot_x-start_trans_x) - line_angular;
          
          //ロボットとLineの距離
          double vertical_diff = std::sin(point_angular) * std::sqrt(std::pow(robot_x - start_trans_x,2) + std::pow(robot_y - start_trans_y,2));

          geometry_msgs::Twist vel;
    
          //scanの値が0.01~1.5の時ループ
          while(min_scan_data_ < 1.5 && min_scan_data_ > 0.01){
            vel.linear.x = 0.0;
            vel.angular.x = 0.0;
            cmd_vel_pub_.publish(vel);
            ros::spinOnce();
            rate.sleep();
          }

          vel.angular.z = alpha * angular_diff + beta * vertical_diff;
          if(vel.angular.z > M_PI/2){
            vel.angular.z = M_PI/2;
          }
          if(vel.angular.z < -M_PI/2){
            vel.angular.z = -M_PI/2;
          }
          vel.linear.x = 0.5;
          cmd_vel_pub_.publish(vel);
          
          ros::spinOnce();
          rate.sleep();
        }

          ROS_INFO("finish line tracking");
          geometry_msgs::Twist vel;
          vel.linear.x = 0.0;
          vel.angular.z = 0.0;
          cmd_vel_pub_.publish(vel);
          line_tracking_flg_ = false;
      }
      ros::spinOnce();
      rate_.sleep();
    }
    if(loop_flg_){
      ROS_INFO("Start waypoint_nav again!");
    }
  } while(ros::ok() && loop_flg_);
  ROS_INFO("Finish waypoint_nav");
}

bool WaypointNav::on_wp(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }

  double wp_x = transformStamped.transform.translation.x;
  double wp_y = transformStamped.transform.translation.y;
  double robot_x = current_waypoint_->pose.position.x;
  double robot_y = current_waypoint_->pose.position.y;
  double dist = std::hypot((wp_x - robot_x), (wp_y - robot_y));

  return dist < dist_err_;
}

void WaypointNav::send_wp(){
  std_srvs::Empty empty;
  while(!clear_costmaps_srv_.call(empty)) {
    ROS_WARN("Resend clear costmap service");
    ros::Duration(0.5).sleep();
   }

  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose.header.stamp = ros::Time::now();
  move_base_goal.target_pose.header.frame_id = world_frame_;
  move_base_goal.target_pose.pose.position = current_waypoint_->pose.position;
  move_base_goal.target_pose.pose.orientation = current_waypoint_->pose.orientation;

  move_base_action_.sendGoal(move_base_goal);


  actionlib::SimpleClientGoalState state_ = move_base_action_.getState();

  while(ros::ok() && (state_ != actionlib::SimpleClientGoalState::ACTIVE)){
    ros::Duration(0.5);
    state_ = move_base_action_.getState();
  }
  last_moved_time_ = ros::Time::now().toSec();
}

void WaypointNav::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg){
  if(cmd_vel_msg->linear.x > -0.001 && cmd_vel_msg->linear.x < 0.001  &&
    cmd_vel_msg->linear.y > -0.001 && cmd_vel_msg->linear.y < 0.001   &&
    cmd_vel_msg->linear.z > -0.001 && cmd_vel_msg->linear.z < 0.001   &&
    cmd_vel_msg->angular.x > -0.001 && cmd_vel_msg->angular.x < 0.001 &&
    cmd_vel_msg->angular.y > -0.001 && cmd_vel_msg->angular.y < 0.001 &&
    cmd_vel_msg->angular.z > -0.001 && cmd_vel_msg->angular.z < 0.001){
    
    ROS_INFO("command velocity all zero");
  }
  else{
    last_moved_time_ = ros::Time::now().toSec();
  }
}

void WaypointNav::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  min_scan_data_ =  get_min_scan_data(scan_msg, 5.0);
  ROS_INFO("min_scan_data=%lf",min_scan_data_);
}

//LaserScan型から90-degree ~ 90+degreeの範囲で最小の値を返す
double WaypointNav::get_min_scan_data(const sensor_msgs::LaserScan::ConstPtr& scan_msg, double degree){

  double one_data_rad = (scan_msg->angle_max - scan_msg->angle_min)/scan_msg->ranges.size();
  double n = std::ceil(M_PI * degree /180 /one_data_rad);
  int start = int(scan_msg->ranges.size()/2 -n);
  int finish = int(scan_msg->ranges.size()/2 +n+1);

  double min_scan_data = 10000;
  for(int i = start; i < finish; i++){
    double temp_scan_data = scan_msg->ranges[i];
    if(temp_scan_data < min_scan_data){
      min_scan_data = temp_scan_data;
    }
  }

  return min_scan_data;
}

bool WaypointNav::startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
  if(suspend_flg_ == true){
    ROS_INFO("Cancel suspend mode!");
    response.success = true;
    response.message = std::string("turn off suspend");
    suspend_flg_ = false;
    last_moved_time_ = ros::Time::now().toSec();
  }
  else{
    ROS_ERROR("Your robot already canceled suspend mode");
    response.success = false;
    return false;
  }
  return true;
}

bool WaypointNav::suspendNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
  if(suspend_flg_ == false){
    ROS_INFO("Go into suspend mode!");
    response.success = true;
    response.message = std::string("turn on suspend");
    suspend_flg_ = true;
  }
  else{
    ROS_ERROR("Your robot is already suspend mode");
    response.success = false;
    return false;
  }
  return true;
}

void WaypointNav::timerCallback(const ros::TimerEvent& e){
  visualize_wp();
}

// This function is not main loop.
// Main loop function's name is run_wp()
void WaypointNav::run(){
  int resend_num = 0;
  send_wp();
  while((resend_num < resend_thresh_) && ros::ok()){
    double time = ros::Time::now().toSec();
    actionlib::SimpleClientGoalState state_ = move_base_action_.getState();
    if(time - last_moved_time_ > wait_time_){
      ROS_WARN("Robot can't reach this waypoint");
      ROS_WARN("Resend this waypoint");
      resend_num++;
      send_wp();
    }
    else if( on_wp() || state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Reach target waypoint!");
      ROS_INFO("Run next waypoint");
      break;
    }
    else if(state_ == actionlib::SimpleClientGoalState::ACTIVE || 
            state_ == actionlib::SimpleClientGoalState::PENDING){
      ros::spinOnce();
      rate_.sleep();
    }
    else{
      ROS_WARN("Robot can't reach this waypoint");
      ROS_WARN("Resend this waypoint");
      resend_num++;
      send_wp();
    }
  }
  if(resend_num >= resend_thresh_){
    ROS_ERROR("Cancel this waypoint because robot can't reach there");
    move_base_action_.cancelAllGoals();
  }
}

void WaypointNav::suspend(){
  run();
  if(suspend_flg_){
    ROS_WARN("Your robot is already suspend mode");
  }
  else{
    ROS_INFO("Your robot will get suspend mode after moving");
    suspend_flg_ = true;
  }
}

void WaypointNav::line_tracking(){
  run();
  
  if(suspend_flg_){
    ROS_WARN("Your robot is already suspend mode");
  }
  else{
    ROS_INFO("Your robot will get suspend mode after moving");
    suspend_flg_ = true;
  } 
    ROS_INFO("Your robot will get line tracking mode after moving");
    line_tracking_flg_ = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_nav");
  WaypointNav wp_nav;
  ros::Rate rate(wp_nav.max_update_rate_);
  bool read_result = wp_nav.read_yaml();
  if(!read_result){
    ROS_ERROR("Waypoint Navigatioin system is shutting down");
    return 1;
  }
  wp_nav.compute_orientation();
  wp_nav.visualize_wp();
  wp_nav.run_wp();

  return 0;
}
