BUVSim: bin/BUVSim.o bin/BUVSimTest.o bin/Controller.o
	g++ -o bin/BUVSim bin/*.o

bin/BUVSim.o: src/BUVSim.cpp
	mkdir -p bin
	g++ -c src/BUVSim.cpp -o bin/BUVSim.o -Iinclude -Iinclude/Eigen3 -std=c++17

bin/BUVSimTest.o: test/BUVSimTest.cpp
	mkdir -p bin
	g++ -c test/BUVSimTest.cpp -o bin/BUVSimTest.o -Iinclude -Iinclude/Eigen3 -std=c++17

clean:
	rm bin/*.o bin/BUVSim

bin/Controller.o: test/Controller.cpp
	mkdir -p bin
	g++ -c test/Controller.cpp -o bin/Controller.o -Iinclude -Iinclude/Eigen3 -std=c++17
