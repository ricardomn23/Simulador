
#include "ControllerVelocity.hpp"

#include <iostream>
#include <cmath>
#include <optional>

#define TRUNCAR(x) (std::remainder(x,360.0))

using namespace std;

ControllerVelocity::ControllerVelocity(){}
ControllerVelocity::~ControllerVelocity(){}



PRECISION ControllerVelocity::calculateVelocity(BUVSimInterface::DState ds){
	
	return std::hypot(ds(0), ds(1), ds(2));	
	
}


