#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "Action.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "State.h"
#include "SweepWorkerStatus.h"
#include <map>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "rclcpp/rclcpp.hpp"

/*
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"
#include <grid_map_msgs/GetGridMap.h>
#include <fstream>
#include <vector>

*/

namespace value_iteration2 {

class ValueIterator {
protected: 
/* value iteration */
	std::vector<State> states_;
	std::vector<Action> &actions_;
	std::vector<std::vector<int> > sweep_orders_;
	uint64_t valueIteration(State &s);

	uint64_t actionCost(State &s, Action &a);
	/*
public:
	void setGoal(double goal_x, double goal_y, int goal_t);
*/
	void valueIterationWorker(int times, int id);
/* calculation */
protected: 
	int toIndex(int ix, int iy, int it);
	bool inMapArea(int ix, int iy);
	void cellDelta(double x, double y, double t, int &ix, int &iy, int &it);
	void noNoiseStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);
	rclcpp::Clock ros_clock_;
/* robot control */
public: /*
	bool endOfTrial(void);
	bool arrived(void);
	Action *posToAction(double x, double y, double t_rad);
	void setCalculated(void);
	bool isCalculated(void);
*/
/* initialization */
	ValueIterator(std::vector<Action> &actions, int thread_num);
	void setMapWithOccupancyGrid(nav_msgs::msg::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	/*
	void setMapWithCostGrid(nav_msgs::msg::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);
protected:
	*/
	void setStateValues(void);
	void setStateTransitionWorker(int it);
	void setStateTransitionWorkerSub(Action &a, int it);
	void setState(const nav_msgs::msg::OccupancyGrid &map, double safety_radius, double safety_radius_penalty);
	void setStateTransition(void);
	void setSweepOrders(void);

/* ros output */
	/*
public:
	bool policyWriter(grid_map_msgs::GetGridMap::Response& response);
	bool valueFunctionWriter(grid_map_msgs::GetGridMap::Response& response);
*/
	void makeValueFunctionMap(nav_msgs::msg::OccupancyGrid &map, int threshold, double yaw_rad);
/* control of value iteration threads */
	std::map<int, SweepWorkerStatus> thread_status_; 
	/*
	void setCancel(void);
	*/
	bool finished(std_msgs::msg::UInt32MultiArray &sweep_times, std_msgs::msg::Float32MultiArray &deltas);
protected:
	std::string status_;

/* parameters */
public: 
	double goal_x_, goal_y_, goal_margin_radius_;
	int goal_t_, goal_margin_theta_;
	int thread_num_;
protected:
	double xy_resolution_, t_resolution_;
	int cell_num_x_, cell_num_y_, cell_num_t_;
	double map_origin_x_;
	double map_origin_y_;
	geometry_msgs::msg::Quaternion map_origin_quat_;
	const static unsigned char resolution_xy_bit_ = 6;
	const static unsigned char resolution_t_bit_ = 6;
public:
	const static unsigned char prob_base_bit_ = resolution_xy_bit_*2+resolution_t_bit_;
	const static uint64_t prob_base_ = 1<<prob_base_bit_;
	const static uint64_t max_cost_ = 1000000000*prob_base_;
/* for local value iteration */
};

//const unsigned char ValueIterator::resolution_xy_bit_ = 6;
//const unsigned char ValueIterator::resolution_t_bit_ = 6;
//const unsigned char ValueIterator::prob_base_bit_ = ValueIterator::resolution_xy_bit_*2+ValueIterator::resolution_t_bit_;
//const uint64_t ValueIterator::prob_base_ = 1<<ValueIterator::prob_base_bit_;
//const uint64_t ValueIterator::max_cost_ = 1000000000*ValueIterator::prob_base_;

}

#endif
