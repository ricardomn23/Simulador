#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <optional>
#include "BUVSim/BUVSim.hpp"

using Point = Eigen::Matrix < PRECISION, 3, 1 >;


class Controller { 

public: 
	Controller();
	~Controller();
	
	std::optional<BUVSimInterface::MotorCommand> goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC);
	
	PRECISION calculateVelocity (BUVSimInterface::DState ds);
	
private:

	PRECISION calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b, PRECISION amp);
	//PRECISION calculateVelocity (BUVSimInterface::DState ds);



};

#endif