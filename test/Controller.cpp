
#include "Controller.hpp"

#include <iostream>
#include <cmath>
#include <optional>

//#define K 0.8
#define K 0.9 //valor de K para a cauda principal
#define KL 1.8 //valor de K para as caudas laterais
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
		
		
				
		angleAxisC = TRUNCAR(angleAxisC);
		anglePoint = TRUNCAR(anglePoint);
		
		cout<<"anglePoint "<<anglePoint<<endl;
		cout<<"angleAxisC "<<angleAxisC<<endl;
		
		if (angleAxisC < anglePoint){
			
			angleError = anglePoint - angleAxisC;
					
		}
		
		else {
			
			angleError = angleAxisC - anglePoint;
			
		}
		
		
		angleError = TRUNCAR(angleError);
		
		PRECISION deflection = angleError;
		//PRECISION deflection = K * angleError;  //K valor de multiplicação ao angulo de erro, originando o valor de deflexão da barbatana
		
		cout<<"deflection1 "<<deflection<<endl;
		
		deflection = TRUNCAR(deflection);
	
		cout<<"deflection2 "<<deflection<<endl;
		
		/*
		//condição para caso deflecção ser maior que 90 graus 
		if (deflection + amp > 90.0){ 
			
			deflection = 90.0 - amp;
			
		}
		
		else if (deflection - amp < -90.0){
			
			deflection = -90.0 + amp;
			
		}
		*/
		
		
		return deflection; //novo valor de deflexão de qualquer barbatana
		
}

PRECISION Controller::calculateVelocity(BUVSimInterface::DState ds){
	
	return std::hypot(ds(0), ds(1), ds(2));	
	
}


std::optional<BUVSimInterface::MotorCommand> Controller::goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand &currentMC){
	
	
	Point sPoint(s(0),s(1),s(2)); //ponto atraves do s (estado do veiculo)
	
	PRECISION l = distance(p, sPoint); //distancia entre o ponto dado e o ponto atual
	PRECISION ls = 0.75; //distancia ao ponto em que o veículo coloca todos os comandos a zero
	
	if (l < 1.0)
		
		return{};
	
	else {
		
		if(l < 20){
			
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
		
		//cauda principal 
		
		PRECISION x = p(0) - s(0); // diferença entre x do p e x do s
		PRECISION y = p(1) - s(1); // diferença entre y do p e y do s
		
		PRECISION psi = s(5) * 180 / M_PI; //calculo do angulo em relação ao eixo do Z (Yaw)
		 
		
		currentMC(1,0) = calculateDeflection (psi, x, y, currentMC(2,0)); //novo valor de deflexão da cauda principal
		
		
		currentMC(1,0) = K * currentMC(1,0); // multiplicação por valor de K para a barbatana Principal
		
		cout<<"angulo BP com K "<<currentMC(1,0)<<endl;
		
		
		//predefinição dos valores de atuação de Frequencia e Amplitude na cauda principal
		if (currentMC(0,0) == 0.0){
			
			currentMC (0,0) = 2500.0;
				
		}
		
		if (currentMC (2,0) == 0.0){
			
			currentMC (2,0) = 30.0;
			
		}
		
		//barbatanas laterais esquerda e direita
		
		PRECISION z = p(2) - s(2); // diferença entre z do p e z do s
		PRECISION phi = s(4) * 180 / M_PI; //calculo do angulo em relação ao eixo do Y (Pitch)
		
		
		PRECISION deflectionLR = calculateDeflection (phi, z, x, currentMC(2,1)); //novo valor de deflexão das barbatanas laterais esquerda e direita
		
		
		
		deflectionLR = KL * deflectionLR; // multiplicação por valor de KL para as Barbatanas Laterais
		
		//deflectionLR = TRUNCAR (deflectionLR); // TESTE
		
		cout<<"deflectionLR "<<deflectionLR<<endl;
		
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