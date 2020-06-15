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
	
	std::optional<BUVSimInterface::MotorCommand> goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC);
	
private:

	ControllerTail controllerTail;
	ControllerSideFins controllerSideFins;
	ControllerVelocity controllerVelocity;

};

#endif