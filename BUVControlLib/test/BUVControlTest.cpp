
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

	Config config("config.txt");


	float T = config.getFloat("T", 0.05);					// Sampling time
	float duration = config.getFloat("duration", 200.0);	 // Simulation time
	float N = ceil(duration/T);   							// Simulation steps


	// Simulator and simulator parameters
	BUV1_Sim buv(T, config.getBool("logging", true));
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
	
	Behaviour::Goal target;
	target << 15.0, 10.0, 5.0;
	
	
	// Mais uma vez, estes valores podiam ser lidos de um ficheiro de configuração.
	
	//act = b.goToPoint(goal, buv.getState(), buv.getDState());
	//act = b.follow(target, buv.getState(), buv.getDState());
	
	float depth = -20.0;
	act = b.goToDepth(depth, buv.getState(), buv.getDState());
	mCommand = c.control(act);
	
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
		
		//act = b.goToPoint(buv.getState(), buv.getDState());
		act = b.goToDepth(depth, buv.getState(), buv.getDState());
		//act = b.follow(target, buv.getState(), buv.getDState());
		mCommand = c.control(act);
	}
	
	logfile.close();
	return 0;
}
