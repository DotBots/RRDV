# RRDV: Robots Rendez-Vous Experiment

This repository contains the firmware for the Ultrasound ranging experiment. 
The solution contains two projects:
	- 00std_rrdv_gateway
	- 00std_rrdv_robot

##Gateway wireless packet frame	
        0              1                2   
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+|
|  gateway ID   | robot bitmask |  robot bitmask |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+|

##Robot wireless packet frame
        0              1                2   
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+|
|   robot ID    |   US echo pulse width in us    |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+|