//SPDX-FileCopyrightText: 2024 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#ifndef _VI_NODE_H__
#define _VI_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <value_iteration2/Action.h>
#include "value_iteration2/ValueIteratorLocal.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

/*
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <iostream>
#include <thread>

#include <grid_map_msgs/GetGridMap.h>
#include <std_msgs/UInt32MultiArray.h>

#include <tf/transform_listener.h>
*/

namespace value_iteration2 {

class ViNode : public rclcpp::Node {

public:
	ViNode();
	~ViNode();

	void init(void);
	void pubValueFunction(void);
	void decision(void);

	void runThreads(void);
private:
	std::vector<Action> *actions_;
	std::shared_ptr<ValueIteratorLocal> vi_;
	rclcpp::TimerBase::SharedPtr decision_timer_;
	rclcpp::TimerBase::SharedPtr value_pub_timer_;

	/*
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::ServiceServer srv_policy_;
	ros::ServiceServer srv_value_;
	*/

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_value_function_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

	void executeVi(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
	/*
	tf::TransformListener tf_listener_;

	shared_ptr<actionlib::SimpleActionServer<value_iteration::ViAction> > as_;

	bool servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
	bool serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

	*/
	void scanReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
	void goalReceived(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

	void setActions(void);
	void setCommunication(void);
	void setMap(void);

	double x_, y_, yaw_;

	//string status_;
	bool online_;

	int cost_drawing_threshold_;
};

}
#endif
