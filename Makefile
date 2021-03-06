BUVSim: bin/BUVSim.o bin/BUVControlTest.o bin/BUVControl.o bin/Config.o
	g++ -o bin/BUVSim bin/*.o

bin/BUVSim.o: BUVSimLib/src/BUVSim.cpp
	mkdir -p bin
	g++ -c BUVSimLib/src/BUVSim.cpp -o bin/BUVSim.o -IBUVSimLib/include -IBUVSimLib/include/Eigen3 -std=c++17

bin/BUVControlTest.o: BUVControlLib/test/BUVControlTest.cpp
	mkdir -p bin
	g++ -c BUVControlLib/test/BUVControlTest.cpp -o bin/BUVControlTest.o -IBUVControlLib/include -IBUVSimLib/include -IBUVSimLib/include/Eigen3 -std=c++17

clean:
	rm bin/*.o bin/BUVSim

bin/BUVControl.o: BUVControlLib/src/BUVControl.cpp
	mkdir -p bin
	g++ -c BUVControlLib/src/BUVControl.cpp -o bin/BUVControl.o -IBUVControlLib/include -IBUVSimLib/include -IBUVSimLib/include/Eigen3 -std=c++17

bin/Config.o: BUVControlLib/src/Config.cpp
	mkdir -p bin
	g++ -c BUVControlLib/src/Config.cpp -o bin/Config.o -IBUVControlLib/include -IBUVSimLib/include -IBUVSimLib/include/Eigen3 -std=c++17
