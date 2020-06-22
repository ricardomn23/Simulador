#include "BUVControl/BUVControl.hpp"


using namespace std;
using namespace Eigen;



/************ CLASS BEHAVIOUR (HIGH LEVEL) ****************/

// Constructor sets default values for variables
Behaviour::Behaviour() :
		T(0.1),
		goTo_K_roll(1.0),
		goTo_K_pitch(1.0),
		goTo_K_heading(1.0),
		goTo_K_speed(1.0),
		cruiseSpeed(1.0),
		reachSpeed(1.0),
		reachedR_squared(1.0)
{}

Behaviour::Actuation Behaviour::stop() {
	return Actuation::Zero();
}
		
// O goToPoint com o parâmetro Goal assume que é a primeira vez que é chamado.
// Pode-se fazer aqui as inicializações que forem necessárias
// (por exemplo, colocar as somas dos termos integrais a 0)
// Depois disso chama-se a função de baixo (sem o parâmetro Goal)
Behaviour::Actuation Behaviour::goToPoint(Goal const &g, State const &state, DState const &dstate) {
	goal = g;
	return goToPoint(state, dstate);
}	

Behaviour::Actuation Behaviour::goToPoint(State const &state, DState const &dstate) {
	Actuation act;

	// roll control: assuming roll_des = 0.0!
	Float err_roll = -TRUNCATE_RAD(state(3));
	// Proportional control:
	act[0] = goTo_K_roll * err_roll;
	
	// pitch control (pitch > 0 ----> diving)
	Float d_xy = sqrt(SQR(goal(1)-state(1)) + SQR(goal(0)-state(0)));
	Float pitch_des = atan2( -(goal(2)-state(2)), d_xy);
	Float err_pitch = TRUNCATE_RAD(pitch_des - state(4));
	// Proportional control:
	act[1] = goTo_K_pitch * err_pitch;

	// heading control  (dHeading > 0 ---> counterclockwise movement)
	Float heading_des = atan2(goal(1)-state(1),goal(0)-state(0));
	Float err_heading = TRUNCATE_RAD(heading_des - state(5));
	// Proportional control:
	act[2] =  goTo_K_heading * err_heading;

	// speed control (using only cruise speed for now...)
	Float curr_speed = sqrt(SQR(dstate(0)) + SQR(dstate(1)) + SQR(dstate(2)));
	Float err_speed = cruiseSpeed - curr_speed;
	// Proportional control:
	act[3] =  goTo_K_speed * err_speed;
	
	return act;
}
	
Behaviour::Actuation Behaviour::standStill(Goal const &goal, State const &state, DState const &dstate) {
	cout << "Behaviour::standStill: NOT IMPLEMENTED YET!" << endl;
	return stop();
}	

Behaviour::Actuation Behaviour::follow(Goal const &goal, State const &state, DState const &dstate) {
	cout << "Behaviour::follow: NOT IMPLEMENTED YET!" << endl;
	return stop();
}	

Behaviour::Actuation Behaviour::goToDepth(Float depth, State const &state, DState const &dstate) {
	cout << "Behaviour::goToDepth: NOT IMPLEMENTED YET!" << endl;
	return stop();
}


/************ CLASS CONTROLLER (LOW LEVEL) ****************/

// Constructor sets default values for variables
Controller::Controller() :
	maxTailAmp(45.0), //maximo valor de amplitude cauda
	maxTailDev(80.0), //maximo valor de deflection cauda
	maxTailFreq(3.0), //maximo valor de frequencia da cauda
	cruiseTailAmp(30.0), //valor standard de amplitude da cauda
	maxSideAmp(45.0), //maximo valor de amplitude das barbatanas alterais
	maxSideDev(80.0),
	maxSideFreq(3.0),
	cruiseSideAmp(30.0),
	cruiseSideFreq(1.0)
{}


Controller::MotorCommand Controller::control(Behaviour::Actuation act) {
	MotorCommand mCommand;
	// Each mCommand column: frequency (Hz), mean value (degrees), amplitude (degrees)
	// First column: tail fin
	// Second column: left fin
	// Third column: right fin

	// no need for rollControl
	//rollControl( act[0], mCommand );
	
	// pitch control:
	pitchControl( act[1], mCommand );

	// heading control:
	headingControl( act[2], mCommand );

	// heading control:
	speedControl( act[3], mCommand );
	
	//chamar o physicalLimits para a cauda
	std::tuple<Float, Float> deflectionAmp = physicalLimits (mCommand(1,0),mCommand(2,0));
	
	mCommand(1,0) = std::get<0>(deflectionAmp); //deflection
	mCommand(2,0) = std::get<1>(deflectionAmp); //amplitude
	
	// If simulator uses frequency in RPM uncomment this:
	//mCommand.row(0) *= 60.0 * 12.0;   // Gearbox with 1:12 ratio
	
	return  mCommand;
}

void Controller::rollControl( Float dRoll, MotorCommand &mCommand ) {
	cout << "Controller::rollControl not implemented yet!!" << endl;
}

void Controller::pitchControl( Float dPitch, MotorCommand &mCommand ) {
	mCommand(1,1) = mCommand(1,2) = - LIMIT(dPitch,-1.0,1.0) * maxSideDev;   // Positive deflection --> negative pitch
	
	mCommand(2,1) = mCommand(2,2) = cruiseSideAmp;
	mCommand(0,1) = mCommand(0,2) = cruiseSideFreq;
	
	std::tuple<Float, Float> deflectionAmp = physicalLimits (mCommand(1,2), mCommand(2,2));
	
	mCommand(1,1) = mCommand(1,2) = std::get<0>(deflectionAmp); //deflection
	mCommand(2,1) = mCommand(2,2) = std::get<1>(deflectionAmp); //amplitude
}

void Controller::headingControl( Float dHeading, MotorCommand &mCommand ) {
	mCommand(1,0) = - LIMIT(dHeading,-1.0,1.0) * maxTailDev;                   // Positive deflection --> negative heading
}

void Controller::speedControl( Float dSpeed, MotorCommand &mCommand ) {
	mCommand(2,0) = cruiseTailAmp;
	mCommand(0,0) = LIMIT(dSpeed,0.0,1.0) * maxTailFreq;
}

std::tuple<Controller::Float, Controller::Float> Controller::physicalLimits( Float deflection, Float amp ) {
	
	Float infLimitFin = deflection - amp;
	
	if (infLimitFin < -85.0){
		
		Float excess = infLimitFin + 85.0;
		
		Float newAmp = amp + excess;
		
		return std::make_tuple (deflection, newAmp);
		
	}
	
	Float supLimitFin = deflection + amp;
	
	if (supLimitFin > 85.0){
		
		Float excess = supLimitFin - 85.0;
		
		Float newAmp = amp - excess;
		
		return std::make_tuple (deflection, newAmp);
		
	}
	
	return std::make_tuple (deflection, amp);
}


