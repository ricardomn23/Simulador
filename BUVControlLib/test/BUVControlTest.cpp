
#include "BUVSim/BUVSim.hpp"
#include "BUVControl/BUVControl.hpp"
#include "Config.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>

using namespace std;
using namespace Eigen;


/**** AUXILIARY FUNCTIONS ******/

std::tuple<std::function<float(float)>, std::function<float(float)>> createFunction(Config &config, std::string const& functionName, std::string const& axis, float defaultValue) {
	if (functionName == "constant") {
		float c = config.getFloat(axis+"Const", defaultValue);
		std::function<float(float)> f = [c](float t) { return c; };
		std::function<float(float)> df = [](float t) { return 0.0; };
		return std::make_tuple (f, df);
	} else if (functionName == "proportional") {
		float a = config.getFloat(axis+"A", defaultValue);
		float c = config.getFloat(axis+"C", defaultValue);
		std::function<float(float)> f = [a,c](float t) { return a * t + c; };
		std::function<float(float)> df = [a](float t) { return a; };
		return std::make_tuple (f, df);
	} else if (functionName == "sin") {
		float b = config.getFloat(axis+"B", defaultValue);
		float w = config.getFloat(axis+"w", defaultValue);
		float c = config.getFloat(axis+"C", defaultValue);
		std::function<float(float)> f = [b,w,c](float t) { return b * sin(w * t) + c; };
		std::function<float(float)> df = [b, w](float t) { return b*w*cos(w*t); };
		return std::make_tuple (f, df);
	} else if (functionName == "cos") {
		float b = config.getFloat(axis+"B", defaultValue);
		float w = config.getFloat(axis+"w", defaultValue);
		float c = config.getFloat(axis+"C", defaultValue);
		std::function<float(float)> f = [b,w,c](float t) { return b * cos(w * t) + c; };
		std::function<float(float)> df = [b, w](float t) { return -b*w*sin(w*t); };
		return std::make_tuple (f, df);
	} else { //default -> constant with defaultValue
		std::function<float(float)> f = [defaultValue](float t) { return defaultValue; };
		std::function<float(float)> df = [](float t) { return 0.0; };
		return std::make_tuple (f, df);
	}
	
}

std::tuple<std::function<float(float)>, std::function<float(float)>> getFunction(Config &config, std::string const& name, float defaultValue) {
	
	cout<<"parametro "<<config.getString("logFilename", "BUV1.log")<<endl;
	cout<<"parametro "<<config.getFloat("yB", defaultValue)<<endl;
	cout<<"configGetS "<<config.getString(name, "")<<endl;
	cout<<"name "<<name<<endl;
	
	return createFunction(config, config.getString(name, ""), name.substr(0,1), defaultValue);
}

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
	b.setGoToKd_speed(config.getFloat("goTo_Kd_speed", 0.1));
	b.setGoToKi_speed(config.getFloat("goTo_Ki_speed", 0.1));
	b.setMinSpeed(config.getFloat("minSpeed", 0.4));
	

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
	
	std::ofstream logfile;
	
	std::string savefile = "target.log";
	logfile.open(savefile);
	if( !logfile.is_open() )
	{
		cout << " could not open \"" << savefile << "\" for logging!" << endl;
		return false;
	}
	
	std::tuple<std::function<float(float)>, std::function<float(float)>> xFunctions = getFunction(config, "xTargetFunction", 0.0);
	std::function<float(float)> xTargetFunction = std::get<0>(xFunctions);
	std::function<float(float)> xDTargetFunction = std::get<1>(xFunctions);
	std::tuple<std::function<float(float)>, std::function<float(float)>> yFunctions = getFunction(config, "yTargetFunction", 0.0);
	std::function<float(float)> yTargetFunction = std::get<0>(yFunctions);
	std::function<float(float)> yDTargetFunction = std::get<1>(yFunctions);
	std::tuple<std::function<float(float)>, std::function<float(float)>> zFunctions = getFunction(config, "zTargetFunction", 0.0);
	std::function<float(float)> zTargetFunction = std::get<0>(zFunctions);
	std::function<float(float)> zDTargetFunction = std::get<1>(zFunctions);
	
	float vx = xDTargetFunction (duration) - xDTargetFunction (0.0);
	float vy = yDTargetFunction (duration) - yDTargetFunction (0.0);
	float vz = zDTargetFunction (duration) - zDTargetFunction (0.0);
	float targetV = sqrt(SQR(vx)+SQR(vy)+SQR(vz));

	
	float t = 0.0;
	
	for(int i=0; i<N; i++)
	{
		target (0) = xTargetFunction(t);
		target (1) = yTargetFunction(t);
		target (2) = zTargetFunction(t);
		
		
		switch (runnigMethod){ //switch para escolher qual o controlador a ser utilizado no ficheiro de configuração
			
			case 1 :
				act = b.goToPoint(goal, buv.getState(), buv.getDState()); // veículo ir para um ponto dado 
				break;
			
			case 2 :
				act = b.follow(target, buv.getState(), buv.getDState(), targetV); //função de veículo seguir outro veículo
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
		
		t += T;
		
		
	}
	
	logfile.close();
	return 0;
}
