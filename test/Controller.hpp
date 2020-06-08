
#include "BUVSim/BUVSim.hpp"

using Point = Eigen::Matrix < PRECISION, 3, 1 >;


class Controller { 

public: 
	Controller();
	~Controller();
	BUVSimInterface::MotorCommand goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC);
	
private:

	PRECISION calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b, PRECISION amp);




};