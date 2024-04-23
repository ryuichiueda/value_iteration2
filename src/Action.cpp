#include "value_iteration2/Action.h"

namespace value_iteration2 {

Action::Action(std::string name, double fw, double rot, int id) : id_(id)
{
	name_ = name;

	delta_fw_ = fw;
	delta_rot_ = rot;

	//_delta_fw_stdev = fabs(fw)*0.1;
	//_delta_rot_stdev = fabs(rot)*0.1;
}

}
