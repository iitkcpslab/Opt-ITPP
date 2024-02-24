Opt-ITPP : Optimal Integrated Task and Path Planner

    This repository contains the source code of Opt-ITPP and some 
    utilities to generate problems and benchmark the algorithm.

    The algorithm Opt-ITPP comprises three planner:
        1. Task Planner : 
            - Generated Task Assignment
            - Utilized Z3 SMT Solver
        2. Path Planner :
            - Based on CBS with Precedence Constraints
        3. Integrated Planner :
            - Handles the interaction between Task and Path 
                Planner and perform end to end execution
                for MultiAgent Multi-Objective Problems
                with capacity and deadline constraints
                and intermediate transfer locations.
    
    Upon successful build, multiple executable binaries will 
    be generated to test each component individually as well.
        a. jsonTester   : Compliance of JSON file with nholman
        b. taskPlanner  : As Defined
        c. mastar       : Multi Label A* tester
        d. cbs			: CBS-PC tester
        e. itmpPlanner*: Primary executable for Opt-ITPP  


1. Pre-Requisites for Optimal Integrated Task and Path Planner
    Build Tested on Ubuntu-20 and 22
    1. boost            : sudo apt install libboost-all-dev
    2. nholman-json     : sudo apt-get install nlohmann-json-dev / nlohmann-json3-dev
    3. cmake            : sudo apt install cmake
    4. z3               :
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
    
3. All executables are present in ./bin/ inside build directory

Executions  :
./bin/taskPlanner --config ../ITMP/examples/problemItmp.json --numZ 6 --opt 1
./bin/jsonTester --config ../ITMP/examples/problemItmp.json
./bin/cbs -c ../ITMP/examples/problemCBS.json --opt 1
./bin/master -c ../ITMP/examples/problemCBS.json


4. For benchmarking, refer to the utilities provided in the ./Utilities directory.


References:
1. https://github.com/Z3Prover/z3
2. https://github.com/HanZhang39/MAPF-PC
3. https://gitlab.com/enricos83/ENHSP-Public
