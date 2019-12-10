#pragma once

#include "common/PidController.hpp"

class LongitudinalControl : msr::airlib::PidController
{
public:
	LongitudinalControl(float p, float i, float d) {
		setPoint(0.0f, p, i, d);
	};

	float Update(float current_velocity, float desired_velocity) {
		float throttle = control(current_velocity - desired_velocity);
		throttle = std::max(-1.0f, std::min(throttle, 1.0f));
		return throttle;
	};
	~LongitudinalControl() {};
};
