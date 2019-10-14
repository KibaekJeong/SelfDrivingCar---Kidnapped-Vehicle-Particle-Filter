# **Kidnapped Vehicle Project**

---

**Building Particle Filter**

The goals / steps of this project are the following:
* Building Particle Filter using C++
* Estimate the location (GPS position) of a moving car using particle filter
* Localize vehicle position and yaw within the specified error value


[//]: # (Image References)
[image1]: ./img/overview.png "overview"
[image2]: ./img/output.png "output"

## Project Introduction
A vehicle has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.
Given GPS estimate of location, new location based on velocity, yaw rate of the vehicle, observation and locations of landmarks, particle filter will try to find where actual vehicle is located at. Particle filter weights to places where vehicle is most likely to be. Afterwards, it re-samples based on likelihood of a particle is where the vehicle is located at, and increases accuracy to localize the vehicle.

## Particle filter system overview
Here is the overview of the system
![alt text][image1]

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

First, run following three commands from the command line

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Then turn on the simulator, which will connect automatically and run the simulation.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)



## Result
Here is the output of the simulation.
Following particle filter used 50 particles.

![alt text][image2]
