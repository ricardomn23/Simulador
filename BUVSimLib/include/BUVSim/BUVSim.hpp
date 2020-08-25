#ifndef bdamas_BUVSIM_H
#define bdamas_BUVSIM_H

#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Dense>


#define PRECISION float

class BUVSimInterface {
public:
	using State = Eigen::Matrix < PRECISION, 6, 1 >;
	using DState = State;
	using DDState = State;
	using Actuation = State;
	using SeaCurr = Eigen::Matrix < PRECISION, 3, 1 >;
	using MotorCommand = Eigen::Matrix < PRECISION, 3, 3 >;

	BUVSimInterface(PRECISION T);
	~BUVSimInterface();

	void reset(State const &state = State::Zero(), DState const &dstate = DState::Zero(),
			   SeaCurr const &curr = SeaCurr::Zero());
	void setSeaCurr( SeaCurr const &curr ) {
		seaCurr = curr; }
	void setBuoyancy( PRECISION b) {
		buoyancy = b; }
	void setMotorCommands(MotorCommand const &mCommand);
	// Each mCommand column: frequency (Hz), mean value (degrees), amplitude (degrees)
	// First column: tail fin
	// Second column: left fin
	// Third column: right fin
	void update();

	bool startLog();
	bool startLog(std::string const &filename);
	bool stopLog();
	bool pauseLog();
	bool resumeLog();

	PRECISION getTime() {
		return t; }
	State const &getState() {
		return s; }
	DState const &getDState() {
		return ds; }
	DDState const &getDDState() {
		return dds; }
	SeaCurr const &getSeaCurr() {
		return seaCurr; }

protected:
	void updateLog();

	void calculateAuxVariables();
	DDState dynamics(State const &s, DState const &ds, Actuation const &tau);
	DState globalCoordinates(DState const &dstate);

	virtual void updateThrustParams(MotorCommand const &mCommand) = 0;
	virtual void updateD(DState const &ds) = 0;
	virtual void updateG(State const &s) = 0;
	virtual Actuation getTau(PRECISION t, DState const &dstate) = 0;

	Eigen::DiagonalMatrix<PRECISION, 6> invM, D;
	Eigen::Matrix<PRECISION, 6, 1> G;
	Eigen::Matrix<PRECISION, 6, 6> LG;

	State s;
	DState ds, v;
	DDState dds;
	Actuation tau;
	SeaCurr seaCurr;
	PRECISION buoyancy;

	PRECISION cPhi,sPhi,cTheta,sTheta,tTheta,cPsi,sPsi;

	PRECISION T,t;

	bool useLog, logging;
	std::string savefile;
	std::string simName;
	std::ofstream logfile;
};

class BUV1_Sim : public BUVSimInterface
{
public:
	BUV1_Sim(PRECISION T, bool log = false);
	BUV1_Sim(PRECISION T, std::string const &filename);

	~BUV1_Sim();

protected:
	void init();
	void initThrustParams();
	void initM();
	void initD();
	void initG();
	void updateThrustParams(MotorCommand const &mCommand);
	void updateD(DState const &ds);
	void updateG(State const &s);
	Actuation getTau(PRECISION t, DState const &dstate);

	//template<int N>
	//PRECISION interpolate(Eigen::Matrix<PRECISION,N,1> const &xVec,
	//					Eigen::Matrix<PRECISION,N,1> const &yVec, PRECISION x);
	PRECISION interpolate(Eigen::Matrix<PRECISION, 7, 1> const &xVec,
		Eigen::Matrix<PRECISION, 7, 1> const &yVec, PRECISION x);

	PRECISION r1, r2, r3, r4, xvxx;
	PRECISION f2wFactor, rpm2fFactor, deg2radFactor;
	PRECISION refAmp;
	Eigen::Matrix<PRECISION, 7, 1> tailRPM, tailAvThrusts, tailAmpThrusts;
	Eigen::Matrix<PRECISION, 7, 1> sideRPM, sideAvThrusts, sideAmpThrusts;

	PRECISION tailAvThrust, tailAmpThrust, tailDefletion, tailW;
	PRECISION leftAvThrust, leftAmpThrust, leftDefletion, leftW;
	PRECISION rightAvThrust, rightAmpThrust, rightDefletion, rightW;

	Eigen::Matrix < PRECISION, 6, 1 > auxD1, auxD2;
	PRECISION auxG;
};

#endif
