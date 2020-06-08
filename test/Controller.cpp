
#include "Controller.hpp"

#include <iostream>

#define K 1
#define TRUNCAR(x) (std::remainder(x,360.0))

using namespace std;

float distance (Point a, Point b){
	
	//calculo da distancia entre o veículo e o ponto dado
	
	return	sqrt(pow((a(0)- b(0)),2) + pow((a(1)- b(1)),2) + pow((a(2)- b(2)),2));
		
	
}

Controller::Controller(){}
Controller::~Controller(){}

PRECISION Controller::calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b, PRECISION amp){
		
		PRECISION anglePoint = atan2 (b,a) * 180 / M_PI; //angulo entre o ponto dado e um dos eixos 
		PRECISION angleError;
		
		cout<<"anglePoint "<<anglePoint<<endl;
		
		angleError = anglePoint - angleAxisC;
		
		if (angleAxisC < anglePoint){
			
			angleError = anglePoint - angleAxisC;
					
		}
		
		else {
			
			angleError = angleAxisC - anglePoint;
			
		}
		
		//cout<<"angleAxisC "<<angleAxisC<<endl;
		
		PRECISION deflection = K * angleError;  //K valor de multiplicação ao angulo de erro, originando o valor de deflexão da barbatana
		
		cout<<"deflection1 "<<deflection<<endl;
		
		//deflection = (int) deflection % 360;
		deflection = TRUNCAR(deflection);
	
	
		//cout<<"angleError "<<angleError<<endl;
		cout<<"deflection2 "<<deflection<<endl;
		/*
		if (deflection + amp > 90.0){
			
			deflection = 90.0 - amp;
			
		}
		
		else if (deflection - amp < -90.0){
			
			deflection = -90.0 + amp;
			
		}
		
		cout<<"deflection3 "<<deflection<<endl;
		*/
		return deflection; //novo valor de deflexão de qualquer barbatana
		
}

/*PRECISION Controller::calculateDeflectionLR (PRECISION currentZ, PRECISION pointZ, PRECISION amp) {
	
	if (currentZ < pointZ) {
		
		return -45.0;
		
	}
	
	else if (currentZ > pointZ) {
		
		return 45.0;
		
	}
	
	else {
		
		return 0.0;
	}
	
	
}
*/





BUVSimInterface::MotorCommand Controller::goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC){
	
	
	//cout<< s.transpose()<<endl; //imprimir o vetor s
	//cout<< p.transpose()<<endl; //imprimir o vetor p
	
	Point sPoint(s(0),s(1),s(2)); //ponto atraves do s (estado do veiculo)
	
	PRECISION l = distance(p, sPoint); //distancia entre o ponto dado e o ponto atual
	PRECISION ls = 0.0;
	
	if (l < ls){
		
		currentMC.col(0) <<  0.0, 0.0, 0.0;
		currentMC.col(1) <<  0.0, 0.0, 0.0;
		currentMC.col(2) <<  0.0, 0.0, 0.0;
		
	}
	
	else {
		//cauda principal 
		PRECISION y = p(1) - s(1); // diferença entre y do p e y do s
		
		PRECISION psi = s(5) * 180 / M_PI; //calculo do angulo em relação ao eixo do Z (Yaw)
		PRECISION x = p(0) - s(0); // diferença entre x do p e x do s 
		
		
		//currentMC(1,0) = calculateDeflection (psi, x, y, currentMC(2,0)); //novo valor de deflexão da cauda principal
		/*
		if (currentMC(0,0) == 0.0){
			
			currentMC (0,0) = 2500.0;
				
		}
		
		if (currentMC (2,0) == 0.0){
			
			currentMC (2,0) = 20.0;
			
		}
		/*
		//barbatanas laterais esquerda e direita
		
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
		PRECISION phi = s(4) * 180 / M_PI; //calculo do angulo em relação ao eixo do X (Pitch)
		PRECISION z = p(2) - s(2); // diferença entre z do p e z do s
		
		
		PRECISION deflectionLR = calculateDeflection (phi, z, x, currentMC(2,1)); //novo valor de deflexão das barbatanas laterais esquerda e direita
		
		//cout<<"deflectionLR "<<deflectionLR<<endl;
		
		currentMC (1 , 1) = - deflectionLR;		
		currentMC (1 , 2) = - deflectionLR;
		
		/*
		PRECISION deflectionLR = calculateDeflectionLR (s(2), p(2), currentMC (2,1));
		
		currentMC (1 , 1) =  deflectionLR;		
		currentMC (1 , 2) =  deflectionLR;
		
		cout<<"deflectionLR "<<deflectionLR<<endl;
		*/
	}
	
	
	return currentMC;
		
}