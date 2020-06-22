
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

Controller::Controller(PRECISION ampTail, PRECISION ampSideFins){
	
	this->ampTail = ampTail;
	this->ampSideFins = ampSideFins;
	
}
Controller::Controller(){
	
	this->ampTail = 20.0;  //valores predefinidos de amplitude para a cauda principal
	this->ampSideFins = 20.0; //valores predefinidos para a amplitude das barbatanas laterais
	
}
Controller::~Controller(){}


std::optional<BUVSimInterface::MotorCommand> Controller::goToPoint(Point p, PRECISION v, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC,  BUVSimInterface::DState ds) {
	
	
	Point sPoint(s(0),s(1),s(2)); //ponto atraves do s (estado do veiculo)
	
	
	PRECISION l = distance(p, sPoint); //distancia entre o ponto dado e o ponto atual
	//PRECISION ls = 5.0; //distancia ao ponto em que o veículo coloca todos os comandos a zero
	
	cout<<"distancia "<<l<<endl;
	
	if (l < 3.5) //distancia do ponto dado a que é terminada a simulação
		
		return{};
	
	else {
		
		//PRECISION newFrequency = this -> controllerVelocity.freqForVelocity (ds, v, currentMC(0,0));
			
		//currentMC(0,0) = newFrequency;
			
		
		
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
		
		//currentMC(1,0) = this -> controllerTail.deflection (p, s, currentMC(2,0));
		
//	cout << currentMC(1,0) << endl;
//	cin.get();
/*
		if( currentMC(1,0) > 40.0)
			currentMC(1,0) = 40.0;
		if( currentMC(1,0) < -40.0)
			currentMC(1,0) = -40.0;
*/		
		//predefinição dos valores de atuação de Frequencia e Amplitude na cauda principal
		if (currentMC(0,0) == 0.0){
			
			currentMC (0,0) = 2500.0;
				
		}
		
		if (currentMC (2,0) == 0.0){
			
			currentMC (2,0) = this->ampTail;
			
		}
		
		//barbatanas laterais esquerda e direita
		
		PRECISION deflectionLR = this -> controllerSideFins.deflection (p, s, currentMC(2,1)); //controlador de altura que funciona
		/*if( deflectionLR > 40.0)
			deflectionLR = 40.0;
		if( deflectionLR < -40.0)
			deflectionLR = -40.0;
		*/
		//PRECISION deflectionLR = this -> controllerHeight.deflection (p, s, currentMC(2,1)); //controlador de altura que nao funciona 
		
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
			
			currentMC (2,1) = this->ampSideFins;
			
		}
		
		if (currentMC (2,2) == 0.0){
			
			currentMC (2,2) = this->ampSideFins;
			
		}
		*/
	}
	
	
	return currentMC;
		
}