//SPDX-FileCopyrightText: 2024 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#include "value_iteration2/vi_node.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace value_iteration2{

ViNode::ViNode() : Node("vi_node")// : private_nh_("~"), yaw_(0.0), x_(0.0), y_(0.0), online_("false")
{
	setActions();
	declare_parameter("global_thread_num", 1);
	declare_parameter("cost_drawing_threshold", 60);

	int thread_num = get_parameter("global_thread_num").as_int();
	RCLCPP_INFO(this->get_logger(),"Global thread num: %d", thread_num);
	cost_drawing_threshold_ = get_parameter("cost_drawing_threshold").as_int();
	vi_.reset(new ValueIteratorLocal(*actions_, thread_num));
}

void ViNode::init(void)
{
	setCommunication();
        setMap();
}

ViNode::~ViNode() 
{
	delete actions_;
}

void ViNode::setMap(void)
{
	declare_parameter("theta_cell_num", 60);
	declare_parameter("safety_radius", 0.2);
	declare_parameter("safety_radius_penalty", 30.0);
	declare_parameter("goal_margin_radius", 0.2);
	declare_parameter("goal_margin_theta", 10);
	declare_parameter("map_type", "occupancy");

	int theta_cell_num = get_parameter("theta_cell_num").as_int();
	double safety_radius = get_parameter("safety_radius").as_double();
	double safety_radius_penalty = get_parameter("safety_radius_penalty").as_double();
	double goal_margin_radius = get_parameter("goal_margin_radius").as_double();
	int goal_margin_theta = get_parameter("goal_margin_theta").as_int();

	std::string map_type = get_parameter("map_type").as_string();
	if(map_type == "occupancy"){
		auto client = create_client<nav_msgs::srv::GetMap>("/map_server/map");
	
		while (!client->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for map.");
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "map server not available, waiting again...");
		}
	
		auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
		auto res = client->async_send_request(req);
		
		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), res) == rclcpp::FutureReturnCode::SUCCESS) {
			vi_->setMapWithOccupancyGrid(res.get()->map, theta_cell_num, safety_radius, safety_radius_penalty,
				goal_margin_radius, goal_margin_theta);
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call map service");
		}


	/*
		while(!ros::service::waitForService("/static_map", ros::Duration(3.0))){
			ROS_INFO("Waiting for static_map");
		}

		ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
		nav_msgs::GetMap::Request req;
		if(not client.call(req, res)){
			ROS_ERROR("static_map not working");
			exit(1);
		}
	
		vi_->setMapWithOccupancyGrid(res.map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);
	}else if(map_type == "cost"){
		while(!ros::service::waitForService("/cost_map", ros::Duration(3.0))){
			ROS_INFO("Waiting for cost_map");
		}

		ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/cost_map");
		nav_msgs::GetMap::Request req;
		if(not client.call(req, res)){
			ROS_ERROR("cost_map not working");
			exit(1);
		}
		for(int i=0;i<100;i++)
			ROS_INFO("%u", (unsigned int)(res.map.data[i] & 0xFF));
	
		vi_->setMapWithCostGrid(res.map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);
	*/
	}else{
		RCLCPP_INFO(this->get_logger(), "NO SUPPORT MAP TYPE");
		exit(1);
	}
}

void ViNode::setCommunication(void)
{
	declare_parameter("online", false);
	online_ = get_parameter("online").as_bool();
	RCLCPP_INFO(this->get_logger(),"Online: %s", online_ ? "true" : "false");

	if(online_){
		pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

		sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1,
				std::bind(&ViNode::scanReceived, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(),"set scan");
		sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1,
				std::bind(&ViNode::goalReceived, this, std::placeholders::_1));
	}

	timer_ = this->create_wall_timer(100ms, std::bind(&ViNode::decision, this));
	pub_value_function_ = create_publisher<nav_msgs::msg::OccupancyGrid>("value_function", 2);

#if 0
	as_->start();
	srv_policy_ = nh_.advertiseService("/policy", &ViNode::servePolicy, this);
	srv_value_ = nh_.advertiseService("/value", &ViNode::serveValue, this);
#endif 
}

void ViNode::setActions(void)
{
	actions_ = new std::vector<Action>();
	actions_->push_back(Action("forward", 0.3, 0.0, 0));
	actions_->push_back(Action("back", -0.2, 0.0, 0));
	actions_->push_back(Action("right", 0.0, -20.0, 0));
	actions_->push_back(Action("rightfw", 0.2, -20.0, 0));
	actions_->push_back(Action("left", 0.0, 20.0, 0));
	actions_->push_back(Action("leftfw", 0.2, 20.0, 0));
}

void ViNode::scanReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	vi_->setLocalCost(msg, x_, y_, yaw_);
}

void ViNode::goalReceived(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) 
{
	executeVi(msg);
}

/*
bool ViNode::servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->policyWriter(response);
	return true;
}

bool ViNode::serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->valueFunctionWriter(response);
	return true;
}

*/
void ViNode::executeVi(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) 
{
	static bool executing = true;
	RCLCPP_INFO(get_logger(), "VALUE ITERATION START");
	auto &ori = msg->pose.orientation;	
	tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	int t = (int)(yaw*180/M_PI);
	vi_->setGoal(msg->pose.position.x, msg->pose.position.y, t);

	/*
	vector<thread> ths;
	for(int t=0; t<vi_->thread_num_; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, vi_.get(), INT_MAX, t));

	if(online_)
		thread(&ValueIteratorLocal::localValueIterationWorker, vi_.get(), 1).detach();
//		for(int t=0;t<1;t++)
//			thread(&ValueIteratorLocal::localValueIterationWorker, vi_.get(), t).detach();


	value_iteration::ViFeedback vi_feedback;

	ros::Rate loop_rate(10);
	while(not vi_->finished(vi_feedback.current_sweep_times, vi_feedback.deltas)){
		as_->publishFeedback(vi_feedback);

		if(as_->isPreemptRequested())
			vi_->setCancel();

		loop_rate.sleep();
	}
	as_->publishFeedback(vi_feedback);

	for(auto &th : ths)
		th.join();

	while(not vi_->endOfTrial() )
		if(as_->isPreemptRequested()){
			vi_->setCancel();

		loop_rate.sleep();
	}

	ROS_INFO("END OF TRIAL");
	value_iteration::ViResult vi_result;
	vi_result.finished = vi_->arrived();
	as_->setSucceeded(vi_result);
*/
}

void ViNode::pubValueFunction(void)
{
	nav_msgs::msg::OccupancyGrid map, local_map;

	vi_->makeValueFunctionMap(map, cost_drawing_threshold_, yaw_);
	pub_value_function_->publish(map);
}

void ViNode::decision(void)
{
	/*
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = 0.00;
	cmd_vel.angular.z = 0.0;
	RCLCPP_INFO(this->get_logger(),"pub");

	pub_cmd_vel_->publish(cmd_vel);
	*/
	/*
	if(not online_)
		return; 

	try{
		tf::StampedTransform trans;
		tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
		tf_listener_.lookupTransform("map", "base_link", ros::Time(0), trans);
		x_ = trans.getOrigin().x();
		y_ = trans.getOrigin().y();
		yaw_ = tf::getYaw(trans.getRotation());
	}catch(tf::TransformException &e){
		ROS_WARN("%s", e.what());
	}

	vi_->setLocalWindow(x_, y_);

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;

	Action *a = vi_->posToAction(x_, y_, yaw_);
	if(a != NULL){
		cmd_vel.linear.x = a->_delta_fw;
		cmd_vel.angular.z = a->_delta_rot/180*M_PI;
	}
	pub_cmd_vel_.publish(cmd_vel);
	*/

	static int step = 0;

	if (step%30 == 0) {
		pubValueFunction();
	}

	step++;
}

}

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<value_iteration2::ViNode>();
//	int step = 0;

	rclcpp::WallRate loop(10);
	node->init();
	rclcpp::spin(node);
	/*
	while (rclcpp::ok()) {
		node->decision();

		//if(step % 30 == 0)
		//	vi_node.pubValueFunction();
		//

		loop.sleep();
		step++;
	}*/
	return 0;
}
