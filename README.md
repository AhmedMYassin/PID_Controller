# CarND-Controls-PID

## Overview
In this project, PID controller is used to control the steering angle and the throttle valve of a simulated vehicle. Twiddle algorithm is used to select the control paramters (P, I, D). 

I used Udaicty silmulator to test and run the project. After tuning the control parameters, the final values are `0.44, 0.03, 2.75`. However these values give the least accuracy, the motion isn't smooth, it's aggressive somehow so I can use also these values `0.3, 0.01, 2` for good accuracy and smooth motion. 

To reduce the time of tuning the parameters, I added termination conditions when the vehicle gets out of the track and when it stucks somewhere. 

To avoid the constant speed problem escpecially in track curves, I used another pid controller for the throttle valve value. This helps alot to control the veicle in curves.

## Tuning Mode

I added the final PID parameters to the code to make it easier for anyone to use the code directly without tuning the parameters, but if you want to see the tuning process, you can uncomment the following line in `main.cpp line 7` to let the code tune the PID parameters.

```
//#define TRAINING_MODE
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
