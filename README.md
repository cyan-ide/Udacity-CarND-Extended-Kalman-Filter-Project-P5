# Extended Kalman Filter Project Starter Code

## Udacity Project 5 Writeup

---

**Extended Kalman Filter Project**

The goals / steps of this project are the following:
* Using pre-made code templates fill out missing parts to implement Extended Kalaman Filter
* Assure the implementation is working with Dataset 1 (and optionally Dataset 2) and properly communicating with the simulator
* Obtained RMSE should not be higher than [0.11,0.11, 0.55,0.55]

---

By large this project was about re-using earlier in-class code and sticking it together. So the main effort was in debuging and making small modification (like zero-value checks/ small value checks) that tilted RMSE towards required value (as the Kalaman filter turns out to be very sensitive to any miscalculations).

**Summary:** The final code gives **RMSE of [0.09,0.08, 0.45, 0.44] for the first Dataset**; and **RMSE of [0.07,0.09,0.42,0.49] for the second dataset**. All were compiled and tested on MacOS (incl. simiulator running locally).

[//]: # (Image References)

[image_1]: ./examples/project5_results.jpg "Project final results"

---

**Key files:**
* src/FusionEKF.cpp, kalman_filter.cpp, tools.cpp  - three files that had missing code to implement (in order: dispatch of measurements to Kalman filter; Kalman filter implementation; RMSE and Jacobian matrix calculation)
* README.md - (this file) - writeup on project coding, challenges and possible improvements. Also, original project description from Udacity.

Additional files:
* install-\*.sh - installation scripts for web sockets library for C (used to communicate between Kalman fitler logic and simulator used to visualise/test)
* data/ - source data for the project
* Docs/ - additional info how the source ata is generated
* ide_profiles/ - profiles for importing src into popoular IDEs
* examples/ - images with results used in this writeup

Additional resources:
* Udacity simulator - complimentary to this project, necessary to test it - https://github.com/udacity/self-driving-car-sim/releases
* Mercedes Utilities - utilites coded for data generation and analysis related to this project assignement -  https://github.com/udacity/CarND-Mercedes-SF-Utilities

---

## Implementation Notes and Challenges enountered:
- the first issues I had were with making everything run correctly. The project instructions are scattered around several class pages and the Readme file, therefore its easy to ommit something. In my case, I didn't notice the comment about tools.cpp and it took me some debuging to figure out that simulator is not working due to server crash happing because tools.cpp function does not return any values. 
- secondly, not as critical tho, the simulator has a well described but also unncessary problem of making the log file to blow out of proportions when server is down. It doesn't cause anything to crash but can take up hard drive space quite quick.
- after I was sure the similuar is running, I started implementing roughtly in order the Udacity project description recommends - started with init focusing on x and P initlisation, afterwards Kalman filter functions and finally the tools functions (with the exception that I had the rmse calculation up-front).
- most of the code is simply copy-paste. Just have to know what to put where. Majority of my issues were due to 'bugs'. I felt this project is mostly a coding task to familiarise with Kalaman filter topic, it doesn't push to do as many experiments and hypotesis testing as the previous projects.

## Coding progress report:
- My first attempt when everything was implemented, compiling and running in simulator gave RMSE = [3.1, 6.2 ,1.1 , 0.9]. Not bad for start but quite a bit away from the target.
- Second attempt of fixing this and playing with precision of variables blew up the error to 100s. Ops, not a good direciton.
- Third attept, going line by line, I discovered I forgot to set current timestemp as previous at the end of initalisation. As a result when second mesurement come it didn't the past time data. When I fixed that was done suddently everything improved, my RMSE was now: [0.12, 0.26, 0.52, 0.65].
- Fourth, final, attempt. I discovered that my Jacobbian matrix had a bug. Even tho it passed in-class exercise, there was a bug of how I was using 'pow' function with 3/2 exponent. After fixing that the RMSE was: [0.09,0.08, 0.45, 0.44]
- At the end I also tested with Dataset 2. The RMSE for that one was: [0.07,0.09,0.42,0.49]. Also meeting Project objectives.


![alt text][image_1]

---
## Project Instructions / Description

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found in the classroom lesson for this project.

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2 (three-term version) or Term 1 (two-term version)
of CarND. If you are enrolled, see the Project Resources page in the classroom
for instructions and the project rubric.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Students have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

