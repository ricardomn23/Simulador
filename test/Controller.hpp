#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <optional>
#include "BUVSim/BUVSim.hpp"
#include "ControllerFins.hpp"
#include "ControllerVelocity.hpp"

//using Point = Eigen::Matrix < PRECISION, 3, 1 >;


class Controller { 

public: 
	Controller();
	~Controller();
	Controller(PRECISION ampTail, PRECISION ampSideFins);
	
	std::optional<BUVSimInterface::MotorCommand> goToPoint(Point p, PRECISION v, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC, BUVSimInterface::DState ds);
	
private:

	PRECISION ampTail;
	PRECISION ampSideFins;
	ControllerTail controllerTail;
	ControllerSideFins controllerSideFins;
	ControllerVelocity controllerVelocity;

	ControllerHeight controllerHeight;
};

#endif