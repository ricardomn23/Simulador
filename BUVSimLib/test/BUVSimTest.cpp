
#include "BUVSim/BUVSim.hpp"

#include <iostream>


using namespace std;
// using namespace Eigen;


int main()
{
	cout << "BUVSim Demo" << endl;

	float T = 1.0/18.0;   // Sampling time
	float N = ceil(40.0/T);   // Simulation steps

	BUV1_Sim buv(T, true);
	Eigen::Matrix < float, 3, 1 > curr(3.0,4.0,-2.0);
	buv.setSeaCurr(curr);
	buv.setBuoyancy(0.0); // Flutuabilidade neutra
	Eigen::Matrix<float,3,3> motorCom;
	// Each mCommand column: frequency (Hz), mean value (degrees), amplitude (degrees)
	// First column: tail fin
	// Second column: left fin
	// Third column: right fin
	motorCom.col(0) << 1.5, 30.0, 20.0;
	motorCom.col(1) << 2.0, 40.0, 20.0;
	motorCom.col(2) << 2.0, 40.0, 20.0;
	buv.setMotorCommands(motorCom);

	cout << "SeaCurr:" << endl;
	cout << curr << endl;
	cout << "Actuation:" << endl;
	cout << motorCom << endl;

	for(int i=0; i<N; i++)
		buv.update();

	return 0;
}
