# ME-495-Quadcopter
This repository has commits from a guided quadcopter build

It wasn't started until the project was significantly advanced, so it constitutes only a partial archive.


The executable "server" should be run on a host computer relaying controller commands to the quad, while the executables "client" and "vive" should be run on the quadcopter's microcontroller. "flight_manager.cpp" is the primary control script, and should be compiled using gcc with the terminal command 
`gcc -o flight_manager flight_manager.cpp -lwiringPi -lncurses -lm`
and then run on the microcontroller to start the quadcopter.

Note that the code here was designed to work with a specific quadcopter chassis, and is unlikely to work with another. It may still be useful as a reference for others attempting their own projects.
