#include "ControllerFins.hpp"

#include <iostream>
#include <cmath>
#include <optional>

#define KZ 1.0 //valor de K para o controlador de profundidade que nao funciona
#define KL 1.8 //valor de K para as caudas laterais
#define K 1.0 //valor de K para a cauda principal
#define TRUNCAR(x) (std::remainder(x,360.0))

using namespace std;


PRECISION ControllerFinsInterface::calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b, PRECISION amp){
		       
		PRECISION anglePoint = atan2(b,a) * 180 / M_PI; //angulo entre o ponto dado e um dos eixos 
		PRECISION angleError;
						
		angleAxisC = TRUNCAR(angleAxisC);  
		anglePoint = TRUNCAR(anglePoint);
		
		cout<<"anguloAoPonto "<<anglePoint<<endl;
		cout<<"anguloAoveículo "<<angleAxisC<<endl;
		
		angleError = anglePoint - angleAxisC;
	
		
		PRECISION deflection = TRUNCAR (angleError);
		//PRECISION deflection = K * angleError;  //K valor de multiplicação ao angulo de erro, originando o valor de deflexão da barbatana
		

		
		cout<<"angulo de erro "<<deflection<<endl;
		
		
		//condição para caso deflecção ser maior que 90 graus 		
		/*
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

PRECISION ControllerTail::deflection(Point p, BUVSimInterface::State s, PRECISION amp){
		
	//cauda principal 
	
	PRECISION x = p(0) - s(0); // diferença entre x do p e x do s
	PRECISION y = p(1) - s(1); // diferença entre y do p e y do s
	
	PRECISION psi = s(5) * 180 / M_PI; //calculo do angulo em relação ao eixo do Z (Yaw)
	 
	//PRECISION err = TRUNCAR(atan2(y,x) - s(5));
	
	
	return TRUNCAR (K * calculateDeflection (psi, x, y, amp)); //novo valor de deflexão da cauda principal
		
}

PRECISION signOf (PRECISION a){
	
	if(a > 0) return 1.0;
	
	else if(a < 0) return -1.0;
	
	else return 0.0;
	
}

ControllerSideFins::ControllerSideFins(){}
ControllerSideFins::~ControllerSideFins(){}

PRECISION ControllerSideFins::deflection(Point p, BUVSimInterface::State s, PRECISION amp){	
	
	//barbatanas laterais esquerda e direita
	
	PRECISION x = p(0) - s(0);
	PRECISION xy = signOf(x) * sqrt(pow(x,2)+ pow(p(1) - s(1),2)); // calculo da hipotenusa entre o x e o y
	PRECISION z = p(2) - s(2); // diferença entre z do p e z do s
	PRECISION phi = s(4) * 180 / M_PI; //calculo do angulo em relação ao eixo do Y (Pitch)
	
	PRECISION deflectionLR = 90.0 - calculateDeflection (phi, z, xy, amp); //novo valor de deflexão das barbatanas laterais esquerda e direita
	
	deflectionLR = TRUNCAR (KL * deflectionLR); // multiplicação por valor de KL para as Barbatanas Laterais
	
	//deflectionLR = TRUNCAR (deflectionLR); // TESTE
	
	cout<<"deflectionLaterais "<<deflectionLR<<endl;
	
	return deflectionLR;
}





ControllerHeight::ControllerHeight(){}
ControllerHeight::~ControllerHeight(){}

PRECISION ControllerHeight::deflection(Point p, BUVSimInterface::State s, PRECISION amp){	//controlador de altura que nao funciona
	
	//barbatanas laterais esquerda e direita
	
	PRECISION z = p(2) - s(2); // diferença entre z do p e z do s
	

	PRECISION deflectionLR = TRUNCAR (KZ * z * 10.0); // multiplicação por valor de KL para as Barbatanas Laterais
	
	
	if (deflectionLR > 70.0){
		
		return 70.0;
		
	}
	
	else if (deflectionLR < -70.0){
		
		return -70.0;
		
	}
	
	return deflectionLR;
		
}