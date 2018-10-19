[image1]: ./doc/1_FlowFilter.png "im1"
[image2]: ./doc/2_PseudoCodeFilter.png "im2"




# Kidnapped Vehicle: A Particle Filter Demo for Vehicle Localization

A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implement a 2 dimensional particle filter in C++ to dynamically localize the robot. A map and some initial localization information are given (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

This project is my solution to Udacity term 2.2 assignment. `src/particle_filter.cpp` contains the implementation and  `src/main.cpp` governs the requirements on accuracy and run time as per [project assignment](https://review.udacity.com/#!/rubrics/747/view).


## Localization Output




## Particle Filter Implementation Notes
The algorithm processing follows the following steps
![alt text][image1]

Below is the pseudocode of the Particle Filter implementation
![alt text][image2]



---
## Dependencies
* Udacity [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* uWebSocketIO. Run `install-mac.sh` or `install-ubuntu.sh` to set up and install  for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

## Running the Code

Once the installation for uWebSocketIO is complete, the main program can be built and ran from the project top directory.

```sh
mkdir build && cd build
cmake .. && make
./particle_filter
```
Alternatively some scripts have been included to streamline this process:
```sh
./clean.sh
./build.sh
./run.sh
```

---
## Data Requirement

Data consists of the following
*  The map (data provided by 3D Mapping Solutions GmbH): `map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns: (1) x position, (2) y position, and (3) landmark id
* Observations and controls provided by Udacity simulator


### Data Flow Between the Filter and the Simulator

Once the Simulator is installed. The main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator is the following

#### INPUT: values provided by the simulator to the c++ program
* Sense noisy position data from the simulator: 
    * ["sense_x"] 
    * ["sense_y"] 
    * ["sense_theta"]

* Get the previous velocity and yaw rate to predict the particle's transitioned state: 
	* ["previous_velocity"]
	* ["previous_yawrate"]

* Receive noisy observation data from the simulator, in a respective list of x/y values: 
	* ["sense_observations_x"]
	* ["sense_observations_y"]

#### OUTPUT: values provided by the c++ program to the simulator
* Best particle values used for calculating the error evaluation: 
	* ["best_particle_x"]
	* ["best_particle_y"]
	* ["best_particle_theta"]

* Optional message data used for debugging particle's sensing and associations for respective (x,y) sensed positions ID label and for respective (x,y) sensed positions
	* ["best_particle_associations"]
	* ["best_particle_sense_x"] <= list of sensed x positions
	* ["best_particle_sense_y"] <= list of sensed y positions

If the filter localizes the object and detect its yaw within the error range (values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`) within the time of 100 seconds, the Simulator will output the message:
```
Success! Your particle filter passed!
```

---
## References
* [Coordinate transformation introduction](https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm)
* [Coordinate transformation formula](http://planning.cs.uiuc.edu/node99.html) 