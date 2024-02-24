1. Install Packages
	1. boost 		: sudo apt install libboost-all-dev
	2. nholman-json		: sudo apt-get install -y nlohmann-json-dev	
	3. z3			:
		a) : git clone https://github.com/Z3Prover/z3.git
		b) : mkdir build && cd build
		c) : cmake ../
		d) : make
		e) : sudo make install
		f) : Check if it is getting installed in /usr/local/lib/cmake/z3/

2. Install ITMP
	1. mkdir build && cd build
	2. cmake ../ITMP
	3. make 
	
3. All executables are present in ./bin/

Executions  :
./bin/taskPlanner --config ../problemItmp.json --numZ 6 --opt 1
./bin/jsonTester --config ../problemItmp.json
./bin/cbs -c ../problemCBS.json --opt 1
./bin/mastar -c ../problemCBS.json
