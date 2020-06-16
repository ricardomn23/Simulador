#ifndef CONTROLLER_VELOCITY_H
#define CONTROLLER_VELOCITY_H


#include "BUVSim/BUVSim.hpp"

using Point = Eigen::Matrix < PRECISION, 3, 1 >;


class ControllerVelocity { 

public: 
	ControllerVelocity();
	~ControllerVelocity();
	
	//std::optional<BUVSimInterface::MotorCommand> goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC);
	
	PRECISION calculateVelocity (BUVSimInterface::DState ds);
	PRECISION freqForVelocity (BUVSimInterface::DState ds, PRECISION goalVelocity, PRECISION frequency);

};

#endif