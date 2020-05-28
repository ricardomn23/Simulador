#include <cmath>

#include "BUVSim/BUVSim.hpp"


#define MIMIC_PIOTR_SIM

using namespace std;

BUVSimInterface::BUVSimInterface(PRECISION T)
: T(T)
{
	simName = "SIMULATOR_NAME_NOT_DEFINED";

	LG.bottomLeftCorner<3,3>().setZero();
	LG.topRightCorner<3,3>().setZero();

	seaCurr.setZero();

	useLog = logging = false;

	cout << "Sampling time: " << T << "s" << endl;
	reset();
}

BUVSimInterface::~BUVSimInterface()
{
	if( useLog )
		stopLog();
}

void BUVSimInterface::reset(State const &state, DState const &dstate, SeaCurr const &curr)
{
	t = 0.0;

	s = state;
	ds = dstate;
	seaCurr = curr;
	v = globalCoordinates(ds);
	v.head<3>() += seaCurr;
}

void BUVSimInterface::setMotorCommands(MotorCommand const &mCommand)
{
	this->updateThrustParams(mCommand);
}

void BUVSimInterface::update()
{
	calculateAuxVariables();

	tau = this->getTau(t,ds);

	dds = dynamics(s, ds, tau);
	ds += dds*T;

	v = globalCoordinates(ds);
	v.head<3>() += seaCurr;

	s += v*T;
	t += T;

	if(logging)
		updateLog();
}

bool BUVSimInterface::startLog()
{
	return startLog( simName + ".log");
}

bool BUVSimInterface::startLog(std::string const &filename)
{
	if( useLog )
	{
		cout << simName << ": already logging to: \"" << savefile << "\", please use stopLog() before." << endl;
		return false;
	}
	else
	{
		savefile = filename;
		logfile.open(savefile);
		if( !logfile.is_open() )
		{
			cout << simName << ": could not open \"" << savefile << "\" for logging!" << endl;
			return false;
		}
		logging = useLog = true;
		updateLog();

		cout << simName << ": logging state to \"" << savefile << "\"" << endl;
		return true;
	}
}

bool BUVSimInterface::stopLog()
{
	if( useLog )
	{
		logfile.close();
		logging = useLog = false;
		cout << simName << ": closed log file \"" << savefile << "\"" << endl;
		return true;
	}
	else
	{
		cout << simName << ": no log file to close!" << endl;
		return false;
	}
}

bool BUVSimInterface::pauseLog()
{
	if( !logging )
	{
		cout << simName << ": already paused!" << endl;
		return false;
	}
	else
	{
		cout << simName << ": logging paused." << endl;
		logging = false;
		return true;
	}
}

bool BUVSimInterface::resumeLog()
{
	if( logging )
	{
		cout << simName << ": already logging!" << endl;
		return false;
	}
	else if( useLog )
	{
		cout << simName << ": resumed logging." << endl;
		logging = true;
		return true;
	}
	else
	{
		cout << simName << ": log file not defined, please use startLog(filename) first!" << endl;
		return false;
	}
}

void BUVSimInterface::updateLog()
{
	logfile << t << " " << s.transpose() << " " << ds.transpose() << endl;
}


void BUVSimInterface::calculateAuxVariables()
{
	// phi = state(3), theta = state(4), psi = state(5)
	cPhi = cos(s(3));
	sPhi = sin(s(3));
	cTheta = cos(s(4));
	sTheta = sin(s(4));
	tTheta = sTheta/cTheta;
	cPsi = cos(s(5));
	sPsi = sin(s(5));
}

BUVSimInterface::DDState BUVSimInterface::dynamics(State const &state, DState const &dstate, Actuation const &tau)
{
	this->updateG(state);
	this->updateD(dstate);

	return this->invM*(tau - this->G - this->D*ds);
}

BUVSimInterface::DState BUVSimInterface::globalCoordinates(DState const &dstate)
{
	LG.topLeftCorner<3,3>() << 	cPsi*cTheta,	cPsi*sTheta*sPhi-sPsi*cPhi,	sPsi*sPhi+cPsi*cPhi*sTheta,
								sPsi*cTheta,	cPsi*cPhi+sPsi*sPhi*sTheta,	sTheta*sPsi*cPhi-cPsi*sPhi,
								-sTheta,		cTheta*sPhi,				cTheta*cPhi;
	LG.bottomRightCorner<3,3>() << 	1.0,sPhi*tTheta,cPhi*tTheta,
									0.0,cPhi,-sPhi,
									0.0,sPhi/cTheta,cPhi/cTheta;

	return LG*dstate;
}


// BUV1:
BUV1_Sim::BUV1_Sim(PRECISION T, bool log)
: BUVSimInterface(T)
{
	init();
	if( log )
		startLog();
}

BUV1_Sim::BUV1_Sim(PRECISION T, string const &filename)
: BUVSimInterface(T)
{
	init();
	startLog(filename);
}

BUV1_Sim::~BUV1_Sim()
{
	cout << simName << ": quiting" << endl;
}

void BUV1_Sim::init()
{
	initThrustParams();
	initM();
	initD();
	initG();

	simName = "BUV1_Sim";

	cout << simName << ": starting" << endl;
}

void BUV1_Sim::initThrustParams()
{
	r1 = 0.1;
	r2 = 0.4;
	r3 = 0.3;
	r4 = 0.13;
#ifdef MIMIC_PIOTR_SIM
	xvxx=0.0;
#else
	xvxx=0.91;
#endif

	refAmp = 20.0; // 20 degrees, just guessing the amplitude of the sinusoids used to
				  // acquire the following data...

	tailRPM << 0.0, 300.0, 600.0, 900.0, 1200.0, 1500.0, 3000.0;
	tailAvThrusts << 0.0, 0.4, 1.2, 2.4, 6.4, 12.8, 18.0;
	tailAmpThrusts << 0.0, 0.6, 1.2, 3.2, 6.2, 8.2, 14.2;

	sideRPM << 0.0, 300.0, 600.0, 900.0, 1200.0, 1500.0, 3000.0;
	sideAvThrusts << 0.0, 0.15, 0.5, 1.2, 3.2, 6.2, 8.0;
	sideAmpThrusts << 0.0, 0.2, 0.4, 1.2, 2.2, 3.2, 5.2;

	rpm2wFactor = M_PI / 30.0 / 12.0;  // Gearbox with 1:12 ratio
	deg2radFactor = M_PI /180.0;

	tailAvThrust = tailAmpThrust = tailDefletion = tailW = 0.0;
	leftAvThrust = leftAmpThrust = leftDefletion = leftW = 0.0;
	rightAvThrust = rightAmpThrust = rightDefletion = rightW = 0.0;
}

void BUV1_Sim::initM()
{
	Eigen::DiagonalMatrix<PRECISION, 6> Mrb,Ma;
	Mrb.diagonal() << 45.0, 45.0, 45.0, 0.73, 7.72, 7.72;
	Ma.diagonal() << 4.5, 59.0, 59.0, 0.0, 11.2, 11.2;
	invM.diagonal() = (Mrb.diagonal() + Ma.diagonal()).cwiseInverse();
}

void BUV1_Sim::initD()
{
	auxD1 << 0.0894, 1.9, 1.9, 0.0, 0.8, 0.7;
	auxD2 << 5.72, 18.58, 18.58, 0.0, 11.0, 10.0;
}

void BUV1_Sim::initG()
{
	auxG = 10.0;
	G.tail<3>().setZero();
}

void BUV1_Sim::updateThrustParams(BUV1_Sim::MotorCommand const &mCommand)
{
	// Each mCommand column: frequency (RPM), mean value (degrees), amplitude (degrees)
	// First column: tail fin
	// Second column: left fin
	// Third column: right fin

	PRECISION scale;

	scale = mCommand(2,0)/refAmp;
	tailAvThrust = interpolate(tailRPM,tailAvThrusts,mCommand(0,0)) * scale;
	tailAmpThrust = interpolate(tailRPM,tailAmpThrusts,mCommand(0,0)) * scale;
	tailDefletion = mCommand(1,0) * deg2radFactor;
	tailW = rpm2wFactor*mCommand(0,0);

	scale = mCommand(2,1)/refAmp;
	leftAvThrust = interpolate(sideRPM,sideAvThrusts,mCommand(0,1)) * scale;
	leftAmpThrust = interpolate(sideRPM,sideAmpThrusts,mCommand(0,1)) * scale;
	leftDefletion = mCommand(1,1) * deg2radFactor;
	leftW = rpm2wFactor*mCommand(0,1);

	scale = mCommand(2,2)/refAmp;
	rightAvThrust = interpolate(sideRPM, sideAvThrusts, mCommand(0,2)) * scale;
	rightAmpThrust = interpolate(sideRPM, sideAmpThrusts, mCommand(0,2)) * scale;
	rightDefletion = mCommand(1,2) * deg2radFactor;
	rightW = rpm2wFactor*mCommand(0,2);
}

void BUV1_Sim::updateD(BUV1_Sim::DState const &dstate)
{
	D.diagonal() = auxD1 + dstate.cwiseAbs().cwiseProduct(auxD2);
}

void BUV1_Sim::updateG(BUV1_Sim::State const &state)
{
	G(0) = auxG*sTheta;
	G(1) = auxG*cTheta*sPhi;
	G(2) = -auxG*cTheta*cPhi;
}

BUV1_Sim::Actuation BUV1_Sim::getTau(PRECISION t, BUV1_Sim::DState const &dstate)
{
	Actuation tau;
	PRECISION cFin, sFin;
	PRECISION aux, drag;

	PRECISION tailThrust = tailAvThrust + tailAmpThrust*sin(tailW*t);
	PRECISION leftThrust = leftAvThrust + leftAmpThrust*sin(leftW*t);
	PRECISION rightThrust = rightAvThrust + rightAmpThrust*sin(rightW*t);

	// Tail:
	cFin = cos(tailDefletion);
	sFin = sin(tailDefletion);

	tau[0] = tailThrust*cFin;
	tau[1] = tailThrust*sFin;
	tau[2] = 0.0;
	tau[3] = 0.0;
	tau[4] = 0.0;
	tau[5] = -tau[1] * (r1 + r2*cFin);

	// Left fin:
	cFin = cos(leftDefletion);
	sFin = sin(leftDefletion);

	aux = leftThrust*cFin;
	tau[0] += aux;
	tau[5] += r4*aux;
	aux = leftThrust*sFin;
	tau[2] += aux;
	tau[4] -= r3*aux;

	// 		Side fins drag (Piotr kind of "hack"):
	drag = xvxx*dstate[0]*abs(sFin);
	tau[0] -= drag;
	tau[4] += drag*r4;

	// Right fin:
	cFin = cos(rightDefletion);
	sFin = sin(rightDefletion);

	aux = rightThrust*cFin;
	tau[0] += aux;
	tau[5] -= r4*aux;
	aux = rightThrust*sFin;
	tau[2] += aux;
	tau[4] -= r3*aux;

	// Side fins drag (Piotr kind of "hack"):
	drag = xvxx*dstate[0]*abs(sFin);
	tau[0] -= drag;
	tau[4] -= drag*r4;

	return tau;
}

//template<int N>
//PRECISION BUV1_Sim::interpolate(Eigen::Matrix<PRECISION,N,1> const &xVec,
//						Eigen::Matrix<PRECISION,N,1> const &yVec, PRECISION x)
//{
PRECISION BUV1_Sim::interpolate(Eigen::Matrix<PRECISION,7,1> const &xVec,
						Eigen::Matrix<PRECISION,7,1> const &yVec, PRECISION x)
{
	int N = 7;
	for(int k = 1; k < N; k++)
		if( x < xVec(k) )
			return yVec(k-1) + (x-xVec(k-1))*(yVec(k)-yVec(k-1))/(xVec(k)-xVec(k-1));

	// Extrapolating too large value
	return yVec(N-2) + (x-xVec(N-2))*(yVec(N-1)-yVec(N-2))/(xVec(N-1)-xVec(N-2));
}
