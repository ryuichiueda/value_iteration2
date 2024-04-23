//SPDX-FileCopyrightText: 2024 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#ifndef _VI_NODE_H__
#define _VI_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <value_iteration2/Action.h>
#include "value_iteration2/ValueIteratorLocal.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/*
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <thread>

#include <grid_map_msgs/GetGridMap.h>
#include <std_msgs/UInt32MultiArray.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
*/

namespace value_iteration2{

class ViNode : public rclcpp::Node {

public:
	ViNode();
	~ViNode();

	/*
	void pubValueFunction(void);
	*/
	void decision(void);
private:
	std::vector<Action> *actions_;
	std::shared_ptr<ValueIteratorLocal> vi_;
	/*
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::ServiceServer srv_policy_;
	ros::ServiceServer srv_value_;
	*/

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
	//Publisher pub_cmd_vel_;
	/*
	ros::Publisher pub_value_function_;
	ros::Subscriber sub_laser_scan_;

	tf::TransformListener tf_listener_;

	shared_ptr<actionlib::SimpleActionServer<value_iteration::ViAction> > as_;

	void executeVi(const value_iteration::ViGoalConstPtr &goal);
	bool servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
	bool serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

	*/
	void scanReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

	void setActions(void);
	void setCommunication(void);
	/*
	void setMap(nav_msgs::GetMap::Response &res);
	*/

	double x_, y_, yaw_;

	//string status_;
	bool online_;

	int cost_drawing_threshold_;
};

}
#endif
