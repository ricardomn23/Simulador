#include "ControllerFins.hpp"

#include <iostream>
#include <cmath>
#include <optional>

#define KL 1.8 //valor de K para as caudas laterais
#define K 0.9 //valor de K para a cauda principal
#define TRUNCAR(x) (std::remainder(x,360.0))

using namespace std;



PRECISION ControllerFinsInterface::calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b){
		       
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
		
		PRECISION deflection = TRUNCAR (angleError);
		//PRECISION deflection = K * angleError;  //K valor de multiplicação ao angulo de erro, originando o valor de deflexão da barbatana
		

		
		cout<<"deflection1 "<<deflection<<endl;
		
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

ControllerTail::ControllerTail(){}
ControllerTail::~ControllerTail(){}

PRECISION ControllerTail::deflection(Point p, BUVSimInterface::State s){
		
	//cauda principal 
	
	PRECISION x = p(0) - s(0); // diferença entre x do p e x do s
	PRECISION y = p(1) - s(1); // diferença entre y do p e y do s
	
	PRECISION psi = s(5) * 180 / M_PI; //calculo do angulo em relação ao eixo do Z (Yaw)
	 
	
	return TRUNCAR (K * calculateDeflection (psi, x, y)); //novo valor de deflexão da cauda principal
		
}

ControllerSideFins::ControllerSideFins(){}
ControllerSideFins::~ControllerSideFins(){}

PRECISION ControllerSideFins::deflection(Point p, BUVSimInterface::State s){	
	
	//barbatanas laterais esquerda e direita
	
	PRECISION x = p(0) - s(0); // diferença entre x do p e x do s
	PRECISION z = p(2) - s(2); // diferença entre z do p e z do s
	PRECISION phi = s(4) * 180 / M_PI; //calculo do angulo em relação ao eixo do Y (Pitch)
	
	
	PRECISION deflectionLR = calculateDeflection (phi, z, x); //novo valor de deflexão das barbatanas laterais esquerda e direita
	
	
	
	deflectionLR = TRUNCAR (KL * deflectionLR); // multiplicação por valor de KL para as Barbatanas Laterais
	
	//deflectionLR = TRUNCAR (deflectionLR); // TESTE
	
	cout<<"deflectionLR "<<deflectionLR<<endl;
	
	return deflectionLR;
			
}