#ifndef VALUE_ACTION_
#define VALUE_ACTION_

//#include "StateTransition.h"
#include <vector>
#include <string>

namespace value_iteration2 {

class Action{
public:
	Action(std::string name, double fw, double rot, int id);

	std::string name_;
	double delta_fw_;  //forward traveling distance[m]
	double delta_rot_;  //rotation[deg]

	int id_;

//	double _delta_fw_stdev;
//	double _delta_rot_stdev;

//	std::vector< std::vector<StateTransition> > _state_transitions; //thetaごとに状態遷移先のリストを保存
};

}

#endif

