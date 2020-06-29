
#include "BUVSim/BUVSim.hpp"
#include "BUVControl/BUVControl.hpp"
#include "Config.hpp"

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;



int main()
{
	cout << "BUV3 Control Demo" << endl;

	Config config("configuration.txt");


	float T = config.getFloat("T", 0.05);					// Sampling time
	float duration = config.getFloat("duration", 200.0);	 // Simulation time
	float N = ceil(duration/T);   							// Simulation steps


	// Simulator and simulator parameters
	BUV1_Sim buv(T, config.getString("logFilename", "BUV1_Sim.log")); //possibilidade de mudar o nome do ficheiro criado com parametros diferentes
	buv.setSeaCurr(config.getVector3f("seaCurr", Eigen::Vector3f(0.0,0.0,0.0)));
	buv.setBuoyancy(config.getFloat("buoyancy", 0.0)); // Flutuabilidade neutra



	// Behaviour (High Level Controller)
	Behaviour b;
	b.setT(T);
	b.setGoToK_heading(config.getFloat("goTo_K_heading", 0.8));
	b.setGoToK_pitch(config.getFloat("goTo_K_pitch", 0.4));
	b.setReachRadius(config.getFloat("reachRadius", 2.0));
	b.setGoToK_roll(config.getFloat("goTo_K_roll", 1.0));
	b.setGoToK_speed(config.getFloat("goTo_K_speed", 1.0));
	b.setCruiseSpeed(config.getFloat("cruiseSpeed", 1.0));
	b.setReachSpeed(config.getFloat("reachSpeed", 1.0));
	b.setGoToKi_heading(config.getFloat("goTo_Ki_heading", 0.1));
	b.setGoToKd_heading(config.getFloat("goTo_Kd_heading", 0.1));
	b.setGoToKi_pitch(config.getFloat("goTo_Ki_pitch", 0.1));
	b.setGoToKd_pitch(config.getFloat("goTo_Kd_pitch", 0.1));
	

	Controller c;
	c.setTailMaxAmp(config.getFloat("tailMaxAmp", 45.0));
	c.setTailMaxDeviation(config.getFloat("tailMaxDeviation", 60.0));
	c.setTailMaxFreq(config.getFloat("tailMaxFreq", 3.0));
	c.setTailCruiseAmp(config.getFloat("tailCruiseAmp", 20.0));
	c.setSideMaxAmp(config.getFloat("sideMaxAmp", 45.0));
	c.setSideMaxDeviation(config.getFloat("sideMaxDeviation", 60.0));
	c.setSideMaxFreq(config.getFloat("sideMaxFreq", 3.0));
	c.setCruiseSideAmp(config.getFloat("cruiseSideAm", 20.0));
	c.setCruiseSideFreq(config.getFloat("cruiseSideFreq", 1.0));

	Behaviour::Actuation act;
	Controller::MotorCommand mCommand;

	
	Behaviour::Goal goal = config.getVector3f("goal", Eigen::Vector3f(0.0,0.0,0.0));
	
	//Behaviour::Goal target;
	//target << 15.0, 10.0, 5.0;
	Behaviour::Goal target = config.getVector3f("followTarget", Eigen::Vector3f(0.0,0.0,0.0));
	
	float depth = config.getFloat("goToDepth", -10.0); //seleção da profundidade pretendida
	
	int runnigMethod = (int) config.getFloat("runnigMethod", 1.0);
	/*
	switch (runnigMethod){
			
		case 1 :
			act = b.goToPoint(goal, buv.getState(), buv.getDState()); // veículo ir para um ponto dado 
			break;
		
		case 2 :
			act = b.follow(target, buv.getState(), buv.getDState()); //função de veículo seguir outro veículo
			break;
			
		case 3 :
			act = b.goToDepth(depth, buv.getState(), buv.getDState()); //controlador de profundidade
			break;
			
		default :
			act = b.goToPoint(goal, buv.getState(), buv.getDState()); // veículo ir para um ponto dado 
	}
		
	mCommand = c.control(act);
	*/
	std::ofstream logfile;
	
	std::string savefile = "target.log";
	logfile.open(savefile);
	if( !logfile.is_open() )
	{
		cout << " could not open \"" << savefile << "\" for logging!" << endl;
		return false;
	}
	
	for(int i=0; i<N; i++)
	{
		
		switch (runnigMethod){ //switch para escolher qual o controlador a ser utilizado no ficheiro de configuração
			
			case 1 :
				act = b.goToPoint(goal, buv.getState(), buv.getDState()); // veículo ir para um ponto dado 
				break;
			
			case 2 :
				act = b.follow(target, buv.getState(), buv.getDState()); //função de veículo seguir outro veículo
				break;
				
			case 3 :
				act = b.goToDepth(depth, buv.getState(), buv.getDState()); //controlador de profundidade
				break;
				
			default :
				act = b.goToPoint(goal, buv.getState(), buv.getDState()); // veículo ir para um ponto dado 
		}
		
		mCommand = c.control(act);
		
		logfile << target.transpose() << endl;
		// Simulate
		buv.setMotorCommands(mCommand);
		buv.update();
		// cout << "State:  " << buv.getState().transpose() << endl;
		// cout << "DState: " << buv.getDState().transpose() << endl;

		// Control
		if( b.hasReachedPoint( buv.getState()) )
			break;
		
		target(0) += 0.2;
		target(1) += 0.0;
		target(2) += 0.0;
		
	}
	
	logfile.close();
	return 0;
}
