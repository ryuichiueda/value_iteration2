#ifndef VALUE_ITERATOR_LOCAL_
#define VALUE_ITERATOR_LOCAL_

#include "ValueIterator.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace value_iteration2 {

class ValueIteratorLocal : public ValueIterator{
public:
	ValueIteratorLocal(std::vector<Action> &actions, int thread_num);

	void setLocalCost(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg, double x, double y, double t);
	/*
	uint64_t valueIterationLocal(State &s);
	void localValueIterationWorker(int id);
	*/

	void setMapWithOccupancyGrid(nav_msgs::msg::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	/*
	Action *posToAction(double x, double y, double t_rad);

	void setLocalWindow(double x, double y);
*/
	void makeLocalValueFunctionMap(nav_msgs::msg::OccupancyGrid &map, int threshold, 
			double x, double y, double yaw_rad);
private: 
	/*
	void localValueIterationLoop(void);

	*/
	bool inLocalArea(int ix, int iy);
	/*
	uint64_t actionCostLocal(State &s, Action &a);
	*/

	int local_ix_min_, local_ix_max_, local_iy_min_, local_iy_max_;
	int local_ixy_range_;
	double local_xy_range_;
};

}

#endif
