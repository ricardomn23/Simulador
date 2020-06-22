
#include "ControllerVelocity.hpp"

#include <iostream>
#include <cmath>

#define TRUNCAR(x) (std::remainder(x,360.0))

#define Kv 6.0

using namespace std;

ControllerVelocity::ControllerVelocity(){}
ControllerVelocity::~ControllerVelocity(){}



PRECISION ControllerVelocity::calculateVelocity(BUVSimInterface::DState ds){
	
	return std::hypot(ds(0), ds(1), ds(2));	
	
}

PRECISION ControllerVelocity::freqForVelocity (BUVSimInterface::DState ds, PRECISION goalVelocity, PRECISION frequency){
	
	PRECISION currentVelocity = calculateVelocity (ds);
	
	cout<<"velocity "<<currentVelocity<<endl;
	
	PRECISION velocityError = (goalVelocity - currentVelocity)/goalVelocity;
	
	PRECISION newFrequency;
	
	if (velocityError > 0){
		
		newFrequency = 2500.0 * velocityError * Kv; // quando erro maior que 0 aumenta a velocidade
		
		if (newFrequency > 2500.0){
			
			newFrequency = 2500.0;
			
		}
	}
	
	else if (velocityError == 0){
		
		return frequency; //quando é atingida a velocidade pretendida, mantem a frequencia
		
	}
	
	else if (velocityError < -1){
		
		newFrequency = 0.0; //quando a velocidade é muito superior à velocidade pretendida, frequancia igual a 0
		
	}
	
	else {
		
		newFrequency = frequency + (9.0/10.0) * velocityError * frequency; // quando a velocidade é ligueiramente superior à pretendida, é reduzida por 0.1 vezes o erro
		
	}
	
	return newFrequency;
	
}
