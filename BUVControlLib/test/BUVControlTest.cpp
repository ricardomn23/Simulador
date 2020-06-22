
#include "BUVSim/BUVSim.hpp"
#include "BUVControl/BUVControl.hpp"

#include <iostream>

using namespace std;
using namespace Eigen;



int main()
{
	cout << "BUV3 Control Demo" << endl;

	float T = 0.05;   					// Sampling time
	float duration = 100.0;		 	// Simulation time
	float N = ceil(duration/T);   	// Simulation steps


	// Simulator and simulator parameters
	BUV1_Sim buv(T, true);
	Eigen::Matrix < float, 3, 1 > curr(0.0,0.0,0.0);
	buv.setSeaCurr(curr);
	buv.setBuoyancy(0.0); // Flutuabilidade neutra
	// Ajustar aqui outros parâmetros do simulador, neste momento usa todos os valores por defeito.
	// Estes parâmetros deviam ser lidos de um ficheiro de configuração, 
	// para evitar estar a compilar código cada vez que se quer experimentar com um valor diferente...


	// Behaviour (High Level Controller)
	Behaviour b;
	b.setT(T);
	b.setGoToK_heading(0.8);
	b.setGoToK_pitch(0.4);
	b.setReachRadius(1.0);
	// etc. Estes parâmetros deviam ser lidos de um ficheiro de configuração, 
	// para evitar estar a compilar código cada vez que se quer experimentar com um valor diferente...

	Controller c;
	// Ajustar aqui os parâmetros do controlador, neste momento usa todos os valores por defeito.
	// Estes parâmetros deviam ser lidos de um ficheiro de configuração, 
	// para evitar estar a compilar código cada vez que se quer experimentar com um valor diferente...


	Behaviour::Actuation act;
	Controller::MotorCommand mCommand;
	Behaviour::Goal goal;
	goal << 20.0, 50.0, 10.0;
//	goal << 60.0, 00.0, -10.0;
	// Mais uma vez, estes valores podiam ser lidos de um ficheiro de configuração.
	
	act = b.goToPoint(goal, buv.getState(), buv.getDState());
	mCommand = c.control(act);
	for(int i=0; i<N; i++)
	{
		// Simulate
		buv.setMotorCommands(mCommand);
		buv.update();
		// cout << "State:  " << buv.getState().transpose() << endl;
		// cout << "DState: " << buv.getDState().transpose() << endl;

		// Control
		if( b.hasReachedPoint( buv.getState()) )
			break;
		
		act = b.goToPoint(buv.getState(), buv.getDState());
		mCommand = c.control(act);
	}

	return 0;
}
