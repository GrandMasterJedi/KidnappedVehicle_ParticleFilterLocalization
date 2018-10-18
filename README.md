# Kidnapped Vehicle: A Particle Filter Demo for Vehicle Localization

A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implement a 2 dimensional particle filter in C++ to dynamically localize the robot. A map and some initial localization information are given (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

This project is my solution to Udacity term 2.2 assignment. `src/particle_filter.cpp` contains the implementation and  `src/main.cpp` governs the requirements on accuracy and run time as per [project assignment](https://review.udacity.com/#!/rubrics/747/view).

## Output of my Implementation









---
## Dependencies
* Udacity [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* uWebSocketIO. Run `install-mac.sh` or `install-ubuntu.sh` to set up and install  for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

## Running the Code

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

---
## Data

### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


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
* Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

* Project [rubric](https://review.udacity.com/#!/rubrics/747/view)