#ifndef CONTROLLER_FINS_INTERFACE_H
#define CONTROLLER_FINS_INTERFACE_H

#include <optional>
#include "BUVSim/BUVSim.hpp"

using Point = Eigen::Matrix < PRECISION, 3, 1 >;


class ControllerFinsInterface { 
	
public:

	virtual PRECISION deflection(Point p, BUVSimInterface::State s) = 0;
	
protected:

	PRECISION calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b);
	



};

class ControllerTail : public ControllerFinsInterface { 

public: 
	ControllerTail();
	~ControllerTail();

	PRECISION deflection(Point p, BUVSimInterface::State s);

};


class ControllerSideFins : public ControllerFinsInterface { 

public: 
	ControllerSideFins();
	~ControllerSideFins();

	PRECISION deflection(Point p, BUVSimInterface::State s);

};


#endif