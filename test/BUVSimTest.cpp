
#include "BUVSim/BUVSim.hpp"
#include "Controller.hpp"
#include <cmath>
#include <iostream>
#include <optional>

using namespace std;
//using namespace Eigen;

//using Point = Eigen::Matrix < PRECISION, 3, 1 >;


int main()
{
	cout << "BUVSim Demo" << endl;

	float T = 1.0/18.0;   // Sampling time
	//float N = ceil(40.0/T);   // Simulation steps
	float N = 2000.0;

	Controller controller (20.0, 20.0); //predefinição da amplitude da cauda e barbatanas laterais
	BUV1_Sim buv(T, true);
	Eigen::Matrix < float, 3, 1 > curr(0.0,0.0,0.0);
	buv.setSeaCurr(curr);
	Eigen::Matrix<float,3,3> motorCom;
	// Each mCommand column: frequency (RPM), mean value (degrees), amplitude (degrees)
	// First column: tail fin
	// Second column: left fin
	// Third column: right fin
	motorCom.col(0) << 0.0, 0.0, 0.0;
	motorCom.col(1) << 1500.0, 0.0, 20.0;
	motorCom.col(2) << 1500.0, 0.0, 20.0;
	buv.setMotorCommands(motorCom);
	//controller.calculateVelocity(buv.getDState());

	cout << "SeaCurr:" << endl;
	cout << curr << endl;
	cout << "Actuation:" << endl;
	cout << motorCom << endl;

	cout <<buv.getState().transpose()<<endl;
	
	PRECISION v = 1.8; //valor desejado de velocidade
	Point p(-20.0,0.0,10.0); //ponto desejado
	
	Point p1(0.0,0.0,0.0);
	
	
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
		
		std::optional<BUVSimInterface::MotorCommand> newMotorCommand;
		BUVSimInterface::State s = buv.getState();
		BUVSimInterface::DState ds = buv.getDState();
		newMotorCommand = controller.goToPoint(p , v, s, currentMotorCom, ds);	
		
		if (!newMotorCommand.has_value())
			break;
	
		buv.setMotorCommands(newMotorCommand.value());
		
		buv.update();

		cout<<"i "<<i<<endl;
/*
		if (i == 1){
			
			cout<<"motorCommand "<<endl;
			cout<<newMotorCommand.value().col (0).transpose()<<endl;
			cout<<newMotorCommand.value().col (1).transpose()<<endl;
			cout<<newMotorCommand.value().col (2).transpose()<<endl;
			
			cout<<"currentMotorCom "<<endl;
			cout<<currentMotorCom.col (0).transpose()<<endl;
			cout<<currentMotorCom.col (1).transpose()<<endl;
			cout<<currentMotorCom.col (2).transpose()<<endl;
		}
*/
		//PRECISION velocity = controller.calculateVelocity(buv.getDState());
		//cout<<"velocity "<<velocity<<endl;

		cout <<buv.getState().transpose()<<endl;
		cout <<"DState "<<buv.getDState().transpose()<<endl;
		
	}
	return 0;
}





