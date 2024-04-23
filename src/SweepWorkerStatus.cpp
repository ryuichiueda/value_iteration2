#include "value_iteration2/ValueIterator.h"
#include <thread>

namespace value_iteration2 {

SweepWorkerStatus::SweepWorkerStatus()
{
	_finished = false;
	_sweep_step = 0;
	_delta = ValueIterator::max_cost_;
}

}
