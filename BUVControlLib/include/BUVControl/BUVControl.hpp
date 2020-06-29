#pragma once

#include "BUVSim/BUVSim.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <tuple>

#define TRUNCATE_DEG(x) (std::remainder(x,360.0))
#define TRUNCATE_RAD(x) (std::remainder(x,2.0*M_PI))
#define SQR(x) (x)*(x)
#define LIMIT(x,inf,sup) ( (x < inf) ? (inf) : ((x > sup) ? (sup) : (x)) )
#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)


/************ CLASS BEHAVIOUR (HIGH LEVEL) ****************/

class Behaviour {
	public:
		using Float = float;							// if double precision is needed change this (and typedefs below)
		using Actuation = Eigen::Vector4f;				// change in [roll, pitch, heading, speed]
		using Goal = Eigen::Vector3f;						// [x_des, y_des, z_des]
		using State = BUV1_Sim::State;  	// [x, y, z, roll, pitch, heading] 
		using DState = BUV1_Sim::DState;  // derivative of State
		
		Behaviour();
		
		// Set Parameters
		void setT(Float t) {		// Sample time
			T = t;}
		void setGoToK_heading(Float K) {
			goTo_K_heading = K; }
		void setGoToK_pitch(Float K) {
			goTo_K_pitch = K; }
		void setGoToK_roll(Float K) {
			goTo_K_roll = K; }
		void setGoToK_speed(Float K) {
			goTo_K_speed = K; }
		void setCruiseSpeed(Float v) {
			cruiseSpeed = v; }
		void setReachSpeed(Float v) {
			reachSpeed = v; }
		void setReachRadius(Float r) {
			reachedR_squared = SQR(r); }
		void setGoToKi_heading(Float Ki) {
			goTo_Ki_heading = Ki; }
		void setGoToKd_heading(Float Kd) {
			goTo_Kd_heading = Kd; }
		void setGoToKd_pitch(Float Kd) {
			goTo_Kd_pitch = Kd; }
		void setGoToKi_pitch(Float Kd) {
			goTo_Kd_pitch = Kd; }
		
		// Behaviours
		Actuation stop();
		Actuation goToPoint(Goal const &g, State const &state, DState const &dstate);
		Actuation goToPoint(State const &state, DState const &dstate);
		Actuation standStill(Goal const &goal, State const &state, DState const &dstate);
		Actuation follow(Goal const &goal, State const &state, DState const &dstate);
		Actuation goToDepth(Float depth, State const &state, DState const &dstate);
		
		bool hasReachedPoint(State const &state) {
			return (goal-state.head<3>()).squaredNorm() < reachedR_squared; }
	
	protected:
		Float T;
		Float goTo_K_roll;
		Float goTo_K_pitch;
		Float goTo_Ki_pitch;
		Float goTo_Kd_pitch;
		Float integral_pitchT;
		Float goTo_K_heading;
		Float goTo_Ki_heading;
		Float goTo_Kd_heading;
		Float integral_headingT;
		Float goTo_K_speed;
		Float cruiseSpeed;
		Float reachSpeed;
		Float reachedR_squared;
		Float err_headingLast;
		Float err_pitchLast;
		
		
		Goal goal;
};


/************ CLASS CONTROLLER (LOW LEVEL) ****************/

class Controller {
	public:
		using Float = float;							// if double precision is needed change this (and typedefs below)
		using Deg = Float;
		using Hertz = Float;
		using FinCommand = Eigen::Vector3f;
		using MotorCommand = BUVSimInterface::MotorCommand;
		
		Controller();
		
		void setTailMaxAmp(Deg amp) {
			maxTailAmp = amp; }
		void setTailMaxDeviation(Deg dev) {
			maxTailDev = dev; }
		void setTailMaxFreq(Hertz freq) {
			maxTailFreq = freq; }
		void setTailCruiseAmp(Deg amp) {
			cruiseTailAmp = amp; }
			
		void setSideMaxAmp(Deg amp) {
			maxSideAmp = amp; }
		void setSideMaxDeviation(Deg dev) {
			maxSideDev = dev; }
		void setSideMaxFreq(Hertz freq) {
			maxSideFreq = freq; }
		void setCruiseSideAmp(Deg amp) {
			cruiseSideAmp = amp; }
		void setCruiseSideFreq(Hertz freq) {
			cruiseSideFreq = freq; }

		MotorCommand control(Behaviour::Actuation act);
		
		// Controllers
		void rollControl( Float dRoll, MotorCommand &mCommand );
		void pitchControl( Float dPitch, MotorCommand &mCommand );
		void headingControl( Float dHeading, MotorCommand &mCommand );
		void speedControl( Float dSpeed, MotorCommand &mCommand );

		std::tuple<Float, Float> physicalLimits( Float deflection, Float amp );

	protected:
		Deg maxTailAmp;
		Deg maxTailDev;
		Hertz maxTailFreq;
		Deg cruiseTailAmp;

		Deg maxSideAmp;
		Deg maxSideDev;
		Hertz maxSideFreq;
		Deg cruiseSideAmp;
		Hertz cruiseSideFreq;
		
		
};

