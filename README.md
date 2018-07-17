# Project Writeup CarND MPC
##### Author: Bugra Turan, 05.07.2018

### Introduction
This is the writeup for the MPC Project of the Udacity CarND term 2. First, we will go through the implementation step-by-step. Afterwards, the parameter search will be discussed and possible ways to improve will be shown.

#### 1) Implementation
Before we go into details of the model I want to describe the process pipeline starting from the telemetry data in the main.cpp.

All relevant data is used from the telemetry like track way point coordinates, (x,y) of the vehicle, the orientation, speed, steering angle, and the throttle. Next the correction value introduced in the lecture Lf was taken with the same value of 2.67.

First, the way point coordinates are transformed into the vehicle frame and a 3rd order polynomial function is fitted through them (only the next 6 coordinates). The cross track error is calculated here as well. Afterwards the initial state is defined and after the first actuator delay of 100ms the state vector is formed. It is composed as followed:

* x_delay -> next x position after delay
* y_delay -> next y position after delay
* psi_delay -> next orientation after delay considering steering and Lf
* v_delay -> next velocity considering current acceleration multiplied with the delay
* cte_delay -> next cross track error calculated from the polyfit slope coefficient
* epsi_delay -> next orientation change velocity

At this point the state vector is fed together with the polyfit coefficients to the MPC and its equations are being solved. The next steering angle value and throttle value are returned by the MPC and send to the simulation. The act delay is considered as for the fact that the next state is interpolated in this time frame.

In the following a will describe the MPC and its model implementation defined in the mpc.cpp.

First, the time step length N is being defined which I have chosen to be 25. In general the computational load increases if this value increases. The time discretization was set to 0.05s and obviously the load is increased if this values is reduced. Wrong choosing of these two values makes the MPC very unstable. These values mean that the optimizer is considering a duration of 1.25s to determine the trajectory. 

Next, the reference CTE, orientation and speed are defined. I have chosen 0, 0, and 45. Furthermore, the start indexes of the state vector parameters are set in accordance with N. 

In the class FG_eval the required additions were made to compute the objective and constraints (and costs). In more detail, the problem is defined by adding first the costs from the reference state to fg[0]. Then the costs coming from the actual next actions delta_start and a_start (or orientation and throttle). Lastly, the costs that originates from two sequential actions is added as well. Please note that the factors 100 and 1000 are used to increase the impact of these objectives.

The lower part of the FG_eval class definition finalizes the remaining fields of the vector fg by setting up the constraints, then looping through all time steps in the horizon N, calculating the polynomial from the fit, and update the state by these values within the discretization dt.

Now that we have our problem definition not much is left to do in the MPC class definition. Basically we first set all constraints on the states like we did in the lecture. Secondly, the FG_eval object is created and the problem is solved in accordance with the state constraints. The solution is pushed into the result   vector and returned.


#### 2) Results
The highest impact on the stability of the MPC do the values N and dt have. Furthermore, the scaling factors in the FG_Eval class for defining the costs do have an strong impact as well. I have just chosen something that give me a proper influence on the performance. Maybe a parameter optimization like we did in the PID project can be helpful here as well.

#### 3) Summary
In summary I found this project quite difficult since there was a lot to implement. Especially the impact of the aforementioned scaling factors helped me a lot. However, at the same time it was very interesting but also showed with term 1 in the mind how easy deep neural network approaches can lead to similar results (at least in this isolated scenario).





