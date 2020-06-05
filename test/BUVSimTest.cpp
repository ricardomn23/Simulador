
#include "BUVSim/BUVSim.hpp"

#include <iostream>

#define K 1

using namespace std;
// using namespace Eigen;

using Point = Eigen::Matrix < PRECISION, 3, 1 >;


float distance (Point a, Point b){
	
	//calculo da distancia entre o veículo e o ponto dado
	
	return	sqrt(pow((a(0)- b(0)),2) + pow((a(1)- b(1)),2) + pow((a(2)- b(2)),2));
		
	
}

PRECISION calculateDeflection(PRECISION angleAxisC, PRECISION a, PRECISION b, PRECISION amp){
		
		PRECISION anglePoint = atan2 (b,a) * 180 / M_PI; //angulo entre o ponto dado e um dos eixos 
		PRECISION angleError;
		
		if (angleAxisC < anglePoint){
			
			angleError = anglePoint - angleAxisC;
					
		}
		
		else {
			
			angleError = angleAxisC - anglePoint;
			
		}
		
		PRECISION deflection = K * angleError;  //K valor de multiplicação ao angulo de erro, originando o valor de deflexão da barbatana
		
		cout<<"deflection1 "<<deflection<<endl;
		
		deflection = (int) deflection % 360;
		
	
	
		//cout<<"angleError "<<angleError<<endl;
		cout<<"deflection2 "<<deflection<<endl;
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


BUVSimInterface::MotorCommand goToPoint(Point p, BUVSimInterface::State s, BUVSimInterface::MotorCommand currentMC){
	
	
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
		
		PRECISION psi = s(5) * 180 / M_PI; //calculo do angulo em relação ao eixo do Z (Yaw)
		PRECISION x = p(0) - s(0); // diferença entre x do p e x do s 
		PRECISION y = p(1) - s(1); // diferença entre y do p e y do s
		
		
		currentMC(1,0) = calculateDeflection (psi, x, y, currentMC(2,0)); //novo valor de deflexão da cauda principal
		
		if (currentMC(0,0) == 0.0){
			
			currentMC (0,0) = 2500.0;
				
		}
		
		if (currentMC (2,0) == 0.0){
			
			currentMC (2,0) = 20.0;
			
		}
		
		//barbatanas laterais esquerda e direita
		/*
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
		
		PRECISION phi = s(3) * 180 / M_PI; //calculo do angulo em relação ao eixo do X (Pitch)
		PRECISION z = p(2) - s(2); // diferença entre z do p e z do s
		
		
		PRECISION deflectionLR = calculateDeflection (phi, y, z, currentMC(2,1)); //novo valor de deflexão das barbatanas laterais esquerda e direita
		
		currentMC (1 , 1) = deflectionLR;		
		currentMC (1 , 2) = deflectionLR;
*/		
	}
	
	
	return currentMC;
		
}







int main()
{
	cout << "BUVSim Demo" << endl;

	float T = 1.0/18.0;   // Sampling time
	//float N = ceil(40.0/T);   // Simulation steps
	float N = 1000.0;

	BUV1_Sim buv(T, true);
	Eigen::Matrix < float, 3, 1 > curr(0.0,0.0,0.0);
	buv.setSeaCurr(curr);
	Eigen::Matrix<float,3,3> motorCom;
	// Each mCommand column: frequency (RPM), mean value (degrees), amplitude (degrees)
	// First column: tail fin
	// Second column: left fin
	// Third column: right fin
	motorCom.col(0) <<  0.0, 0.0, 0.0;
	motorCom.col(1) << 0.0, 0.0, 0.0;
	motorCom.col(2) << 0.0, 0.0, 0.0;
	buv.setMotorCommands(motorCom);

	cout << "SeaCurr:" << endl;
	cout << curr << endl;
	cout << "Actuation:" << endl;
	cout << motorCom << endl;

	cout <<buv.getState().transpose()<<endl;
	
	Point p(-10.0,-10.0,0.0);
	
	Point p1(0.0,0.0,0.0);
	
	float dist = distance (p1,p);
	
	cout<<"distancia "<<dist<<endl;
	
	BUVSimInterface::MotorCommand currentMotorCom;
	
	currentMotorCom(0,0) = motorCom(0,0);
	currentMotorCom(0,1) = motorCom(0,1);
	currentMotorCom(0,2) = motorCom(0,2);
	currentMotorCom(1,0) = motorCom(1,0);
	currentMotorCom(1,1) = motorCom(1,1);
	currentMotorCom(1,2) = motorCom(1,2);
	currentMotorCom(2,0) = motorCom(2,0);
	currentMotorCom(2,1) = motorCom(2,1);
	currentMotorCom(2,2) = motorCom(2,2);
	
	for(int i=0; i<N; i++){
		
		//cout<<"goToPoint "<<p(0)<<endl;
		BUVSimInterface::MotorCommand newMotorCommand;
		BUVSimInterface::State s = buv.getState();
		newMotorCommand = goToPoint(p , s, currentMotorCom);	
	
		buv.setMotorCommands(newMotorCommand);
		buv.update();

		cout <<buv.getState().transpose()<<endl;

	}
	return 0;
}





