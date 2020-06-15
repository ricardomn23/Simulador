
#include "Controller.hpp"

#include <iostream>
#include <cmath>
#include <optional>

#define TRUNCAR(x) (std::remainder(x,360.0))

using namespace std;

float distance (Point a, Point b){
	
	//calculo da distancia entre o veículo e o ponto dado
	
	return	sqrt(pow((a(0)- b(0)),2) + pow((a(1)- b(1)),2) + pow((a(2)- b(2)),2));
		
}

Controller::Controller(){ 
/*
	ControllerTail tail;
	ControllerSideFins sideFins;
	ControllerVelocity velocity;
	
	this -> controllerTail = tail;
	this -> controllerSideFins = sideFins;
	this -> controllerVelocity = velocity;
*/
}
Controller::~Controller(){}

std::optional<BUVSimInterface::MotorCommand> Controller::goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC){
	
	
	Point sPoint(s(0),s(1),s(2)); //ponto atraves do s (estado do veiculo)
	
	PRECISION l = distance(p, sPoint); //distancia entre o ponto dado e o ponto atual
	PRECISION ls = 0.75; //distancia ao ponto em que o veículo coloca todos os comandos a zero
	
	if (l < 1.0)
		
		return{};
	
	else {
		/*
		if(l < 0){
			
			currentMC(0,0) = 0.0;
			currentMC(0,1) = 0.0;
			currentMC(0,2) = 0.0;
			currentMC(1,0) = 0.0;
			currentMC(1,1) = 0.0;
			currentMC(1,2) = 0.0;
			currentMC(2,0) = 0.0;
			currentMC(2,1) = 0.0;
			currentMC(2,2) = 0.0;
			
			
			return currentMC;
		}
		*/
		//cauda principal 		 
		
		currentMC(1,0) = this -> controllerTail.deflection (p, s);
		
		/*
		//predefinição dos valores de atuação de Frequencia e Amplitude na cauda principal
		if (currentMC(0,0) == 0.0){
			
			currentMC (0,0) = 2500.0;
				
		}
		
		if (currentMC (2,0) == 0.0){
			
			currentMC (2,0) = 30.0;
			
		}
		*/
		//barbatanas laterais esquerda e direita
		
		PRECISION deflectionLR = this -> controllerSideFins.deflection (p, s);
		
		currentMC (1 , 1) =  deflectionLR;		
		currentMC (1 , 2) =  deflectionLR;
	
		/*
		//predefinição dos valores de atuação de Frequencia e Amplitude nas caudas laterias
		if (currentMC (0,1) == 0.0){
			
			currentMC (0,1) = 1500.0;
			
		}
		if (currentMC (0,2) == 0.0){
			
			currentMC (0,2) = 1500.0;
			
		}
		
		if (currentMC (2,1) == 0.0){
			
			currentMC (2,1) = 20.0;
			
		}
		
		if (currentMC (2,2) == 0.0){
			
			currentMC (2,2) = 20.0;
			
		}
		*/
	}
	
	
	return currentMC;
		
}