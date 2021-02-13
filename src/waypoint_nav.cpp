#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"

#include <vector>
#include <list>
#include <string>
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
  double max_update_rate_;
  bool read_yaml();
  void compute_orientation();
  bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool suspendNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  void run();
  void run_wp_once();
private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
  std::list<Waypoints> waypoints_;
  decltype(waypoints_)::iterator current_waypoint_;
  std::string robot_frame_, world_frame_;
  std::string filename_;
  int resend_num_;
  bool loop_flg_;
  bool suspend_flg_;
  double dist_err_, last_moved_time_;
  ros::Rate rate_;
  ros::ServiceServer start_server_, suspend_server_; 
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher wp_pub_;
  ros::ServiceClient clear_costmaps_srv_;
};

WaypointNav::WaypointNav() :
    move_base_action_("move_base", true),
    rate_(1.0),
    last_moved_time_(0.0),
    loop_flg_(false),
    suspend_flg_(true)
{
  nh_.param("waypoint_nav/robot_frame", robot_frame_, std::string("/base_link"));
  nh_.param("waypoint_nav/world_frame", world_frame_, std::string("/map"));

  nh_.param("waypoint_nav/max_update_rate", max_update_rate_, 1.0);
  ros::Rate rate_(max_update_rate_);

  nh_.param("waypoint_nav/filename", filename_, filename_);
  nh_.param("waypoint_nav/dist_err", dist_err_, 1.0);

  nh_.param("waypoint_nav/loop_flg", loop_flg_, false);


  start_server_ = nh_.advertiseService("start_wp_nav", &WaypointNav::startNavigationCallback, this);
  suspend_server_ = nh_.advertiseService("suspend_wp_nav", &WaypointNav::suspendNavigationCallback, this);
//  cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &WaypointNav::cmdVelCallback, this);
//  clear_costmaps_srv_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
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
        ROS_ERROR("function is set by default (run) because function is not set in yaml");
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
  for(it = waypoints_.begin(), it2 = std::next(waypoints_.begin()); it != waypoints_.end(); it++, it2++){
    if(it2 != waypoints_.end()){
      goal_direction = atan2((it2)->pose.position.y - (it)->pose.position.y,
                              (it2)->pose.position.x - (it)->pose.position.x);
      it->pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
    }
    else{
      // set direction which is same as previous one
      it->pose.orientation = std::prev(it)->pose.orientation;
    }
  }
}

bool WaypointNav::startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
  if(suspend_flg_ == true){
    ROS_INFO("Cancel suspend mode!");
    response.success = true;
    response.message = std::string("turn off suspend");
    suspend_flg_ = false;
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

void WaypointNav::run(){
  while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && ros::ok()){
      ROS_INFO("Waiting...");
  }

  while(ros::ok()){
    // If loop_flg_ is true, do_while loop infinitely
    do{
      for(current_waypoint_ = waypoints_.begin(); (current_waypoint_ != waypoints_.end()) && ros::ok(); current_waypoint_++){
        if(!suspend_flg_){
          run_wp_once();
        }
        ros::spinOnce();
        rate_.sleep();
      }
    } while(loop_flg_);
  }
}

void WaypointNav::run_wp_once(){
  resend_num_ = 0;
  while((resend_num_ < 3) && ros::ok()){
//    pub_wp();
    actionlib::SimpleClientGoalState state_ = move_base_action_.getState();
    if(state_ == actionlib::SimpleClientGoalState::ACTIVE){
      ros::spinOnce();
      rate_.sleep();
    }
    else if(state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Run next waypoint");
      break;
    }
    else{
      resend_num_++;
    }
  }
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
  wp_nav.run();

  return 0;
}
