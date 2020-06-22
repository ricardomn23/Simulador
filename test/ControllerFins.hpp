#ifndef CONTROLLER_FINS_INTERFACE_H
#define CONTROLLER_FINS_INTERFACE_H

#include <optional>
#include "BUVSim/BUVSim.hpp"

using Point = Eigen::Matrix < PRECISION, 3, 1 >;


class ControllerFinsInterface { 
	
public:

	virtual PRECISION deflection(Point p, BUVSimInterface::State s, PRECISION amp) = 0;
	
protected:

	PRECISION calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b, PRECISION amp);
	



};

class ControllerTail : public ControllerFinsInterface { 

public: 
	ControllerTail();
	~ControllerTail();

	PRECISION deflection(Point p, BUVSimInterface::State s, PRECISION amp);

protected:
	
	
};


class ControllerSideFins : public ControllerFinsInterface { 

public: 
	ControllerSideFins();
	~ControllerSideFins();

	PRECISION deflection(Point p, BUVSimInterface::State s, PRECISION amp);

protected:
	

};

class ControllerHeight : public ControllerFinsInterface { 

public: 
	ControllerHeight();
	~ControllerHeight();

	PRECISION deflection(Point p, BUVSimInterface::State s, PRECISION amp);
	

};
#endif