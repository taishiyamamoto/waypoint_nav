/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Daiki Maekawa and Chiba Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_srvs/Trigger.h>

#include <vector>
#include <fstream>
#include <string>

typedef struct Waypoints{
    geometry_msgs::Pose pose;
    std::string function;
}Waypoints;

class WaypointsSaver{
public:
    WaypointsSaver() : 
        filename_("waypoints.yaml"),
        tfListener_(tfBuffer_)
    {
        waypoints_viz_sub_ = nh_.subscribe("waypoints_viz", 1, &WaypointsSaver::waypointsVizCallback, this);
        waypoints_joy_sub_ = nh_.subscribe("joy", 1, &WaypointsSaver::waypointsJoyCallback, this);

        save_server_ = nh_.advertiseService("save_waypoints", &WaypointsSaver::saveWaypointCallback, this);
        create_server_ = nh_.advertiseService("create_waypoints", &WaypointsSaver::createWaypointCallback, this);

        ros::NodeHandle private_nh("~");
        private_nh.param("filename", filename_, filename_);
        private_nh.param("save_joy_button", save_joy_button_, 0);
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("world_frame", world_frame_, std::string("map"));
    }

    void waypointsJoyCallback(const sensor_msgs::Joy &msg){
        static ros::Time saved_time(0.0);
        //ROS_INFO_STREAM("joy = " << msg);
        if(msg.buttons[save_joy_button_] == 1 && (ros::Time::now() - saved_time).toSec() > 3.0){
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::Pose pose;
            std::string function;

            try{
                transformStamped = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0));
                pose.position.x = transformStamped.transform.translation.x;
                pose.position.y = transformStamped.transform.translation.y;
                pose.position.z = transformStamped.transform.translation.z;
                function = "run";

                waypoints_.push_back({pose, function});
                saved_time = ros::Time::now();
            }
            catch (tf2::TransformException &ex){
                ROS_WARN("%s", ex.what());
            }
        }
    }
    
    void waypointsVizCallback(const geometry_msgs::PointStamped &msg){
        ROS_INFO_STREAM("point = " << msg);

        geometry_msgs::Pose pose;
        std::string function;

        pose.position.x = msg.point.x;
        pose.position.y = msg.point.y;
        pose.position.z = msg.point.z;
        function = "run";

        waypoints_.push_back({pose, function});
    }

    bool createWaypointCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
        static ros::Time saved_time(0.0);
        ROS_INFO("service served create_waypoints");
        if((ros::Time::now() - saved_time).toSec() > 3.0){
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::Pose pose;
            std::string function;

            try{
                transformStamped = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0));
                pose.position.x = transformStamped.transform.translation.x;
                pose.position.y = transformStamped.transform.translation.y;
                pose.position.z = transformStamped.transform.translation.z;
                function = "run";

                waypoints_.push_back({pose, function});
                saved_time = ros::Time::now();
                response.success = true;
                return true;
            }
            catch (tf2::TransformException &ex){
                ROS_WARN("%s", ex.what());
                response.success = false;
                return false;
            }
        }
        else{
            response.success = false;
            return false;
        }
    }

    bool saveWaypointCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
        if (save_yaml()) {
            response.success = true;
            return true;
        } else {
            response.success = false;
            return false;
        }
    }
    
    bool save_yaml(){
        std::ofstream ofs(filename_.c_str(), std::ios::out);
        if(ofs){
            ofs << "waypoints:" << std::endl;

            for(const auto& p : waypoints_){
                ofs << "    " << "- point:" << std::endl;
                ofs << "        x: " << p.pose.position.x << std::endl;
                ofs << "        y: " << p.pose.position.y << std::endl;
                ofs << "        z: " << p.pose.position.z << std::endl;
                ofs << "        function: " << p.function << std::endl;
            }

            ROS_INFO_STREAM("write success");
            ofs.close();
            return true;
        }
        else{
            ROS_ERROR("failed to write \"%s\"", filename_.c_str());
            ofs.close();
            return false;
        } 
        ofs.close();

    }
    
    void run(){
        ros::spin();
    }
    
private:
    ros::Subscriber waypoints_viz_sub_;
    ros::Subscriber waypoints_joy_sub_;
    ros::ServiceServer save_server_;
    ros::ServiceServer create_server_;
    std::vector<Waypoints> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    int save_joy_button_;
    ros::NodeHandle nh_;
    std::string filename_;
    std::string world_frame_;
    std::string robot_frame_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "waypoints_saver");
    WaypointsSaver saver;
    saver.run();

    return 0;
}
