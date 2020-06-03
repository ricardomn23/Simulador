BUVSim: bin/BUVSim.o bin/BUVSimTest.o
	g++ -o bin/BUVSim bin/*.o

bin/BUVSim.o: src/BUVSim.cpp
	mkdir -p bin
	g++ -c src/BUVSim.cpp -o bin/BUVSim.o -Iinclude -Iinclude/Eigen3 -std=c++11

bin/BUVSimTest.o: test/BUVSimTest.cpp
	mkdir -p bin
	g++ -c test/BUVSimTest.cpp -o bin/BUVSimTest.o -Iinclude -Iinclude/Eigen3 -std=c++11

clean:
	rm bin/*.o bin/BUVSim

