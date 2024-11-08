# Calibration of Front-Tractor Tricycle

**Final Project for Probabilistic Robotics Course**

#### Student:
- Charlotte Ludovica Primiceri 2021543

## Dataset

**Input Data:**

- Initial values of [ Ksteer, Ktraction, axis_lenght, steer_offset] = [ 0.1, 0.0106141, 1.4, 0]
  - Ksteer: radians for one tick of the absolute encoder
  - Ktraction: meters for one tick of the absolute encoder
  - Steer Offset: correction on the direction of the front wheel in order to go forward (zero angle)
    
- Position of the Laser w.r.t. the base link:
  
  translation: [1.5, 0, 0]
  
  rotation: [0, 0, 0, 1]\
 
  so the laser position and orientation w.r.t. the base link is [x y theta]=[1.5 0 0]

For a certain time step we have:
- ticks of the absolute encoder for the steering, with maximum value for a joint of 8192;
- tick of the incremental encoder for the traction, with maximum value for a joint of 5000;
- model pose;
- tracker pose of the sensor. 


## Task

Find the output:
- 2D position of the sensor w.r.t the base link
- The kinematic parameters [ ksteer, ktraction, steer_offset, base_line ]
- From the ground truth of the uncalibrated odometry of the front wheel: 

  ![Figure_1](https://github.com/CharlottePrimiceri/ProbabilisticRobotics/blob/main/04-Calibration/images/uncalibrated_odometry_ground_truth.png)

  find the correct trajectory for the laser attached to the front wheel. The ground truth is given in python:
  
  ![Figure_2](https://github.com/user-attachments/assets/ca34aed1-7f20-48aa-b1fd-7660e9ba3524)

## Solution

### Dataset:
- Time Values:

  Because of the floating point, is better to reinitialize the time, from 0, to have more precise increment value of time. In fact, in matlab, if we compute the eps(number_a), the error that can be computed between number_a and the minum computable consecutive one number_b, we obtain:\
  eps(1.6e+09) = 2.38e-07.\ 
  So if it occurs  an increment of the last two digits in 1668091584.821040869 (example of our dataset) then it would be lost. 

- How to deal with Overflow?

  As it is suggested, the reading of the incremental encoder, is stored in an uint32 variable which has a maximum range of 4294967295. 
  So, if the previous value of tick is greater than the next one and it cannot be considered as a backword motion, there is overflow. To avoid that, in this case change the increment of ticks as:\
  ``` 
  overflow_delta = overflow_max_value - previous_tick_value + new_tick_value
  ```
  otherwhise maintain the actual difference. So the two conditions that occur in case of overflow is that the current incremental value is lesser than the previous incremental value AND the overflow_delta is lesser than the difference between the previous incremental value and the current one (this is the case for which this can't be a backward motion).
  For example, considering two cases where the previous value is bigger than the second: \
  1771th sample: enc_value=10005756\
  1772th sample: enc_value=9989426\
  This should be a simple backward motion\
  current - previous = -16330\
  And in fact overflow-previous+current is way to bigger than 16330, so it's not the case of overflow.\
  While considering:\
  67th sample: enc_value=4294962835\
  68th sample: enc_value=526\
  overflow-previous+current is lesser than the difference preovious-current, so we have overflow. 

- True laser pose trajectory in octave:
   ![Figure_3](https://github.com/CharlottePrimiceri/ProbabilisticRobotics/blob/main/04-Calibration/images/true_traj_octave.png) 

### Laser pose w.r.t base link

- The laser transformation w.r.t. the reference frame of the robot is given by the transformation:\
  (1) $T_{laser}^{reference} = T_{rear_pose}^{reference} \cdot T_{laser}^{baselink}$\
  The last term is the pose of the laser with respect the rear wheel frame.\
  (2) $T_{laser}^{baselink} = (T_{laser}^{baselink})^{-1} \cdot T_{laser}^{reference}$

### Kinematic Model of Front-Tractor Tricycle
  
- Drawing the model of the tricycle we can obtain the pose of the front wheel.
  <img src="https://github.com/CharlottePrimiceri/ProbabilisticRobotics/blob/main/04-Calibration/images/tricycle.jpg" width="400" height="350">

  The configuration state is q = [$x_{front}$; $y_{front}$; $\theta$ ; $\psi$].  
  (3) $x_{front} = x_{rear} + cos(\theta) $\
  (4) $y_{rear} = y_{rear} + sin(\theta) $\
  Consider the pure rolling constraint for the front wheel and for the rear wheels consider the midpoint of the axe that connects them:\
  (5) $\dot x_{front} sin(\theta + \psi) - \dot y_{front} cos(\theta + \psi) = 0$\
  (6) $\dot x_{rear} sin(\theta) - \dot y_{rear} cos(\theta) = 0$\
  Substitute the first two equations in the last one:\
  (7) $\dot x_{front} sin(\theta) - \dot y_{front} cos(\theta ) + \dot{\theta} l = 0$\
  The relationship between the state and the input is defined by the kinematic model:\
  (8) $\dot q = g_{1} u_{1} + g_{2} u_{2}$\ 
  The two input are respectively the driving velocity, v, and the steering velocity, w, of the front wheel.\
  We need to find $g_{1}$ and $g_{2}$, vector basis of $\dot q$, so that $A^{T}(q) \dot q = 0$, from eq. (7), is satisfied.\

  <img src="https://github.com/CharlottePrimiceri/ProbabilisticRobotics/blob/main/04-Calibration/images/kin_model.jpg" width="500" height="500">

  <img src="https://github.com/CharlottePrimiceri/ProbabilisticRobotics/blob/main/04-Calibration/images/kin_model_2.jpg" width="350" height="300">

  Given an initial state q = [0; 0; 0; 0], at each time add the previous value of the state to the current one.\
  The driving velocity is computed through the incremental encoder information by multiplying the number of its ticks, in each time stamp, for the value of meters corresponding to one single tick:
  ``` 
  traction_front = traction_incremental_ticks * (ticks_to_meters / (traction_max))
  ```
  While the steering velocity is computed through the absolute encoder by multiplying the number of its ticks, in each time stamp, to the value of radians (converted from revolution to radians with a factor 2pi) corresponding to one single tick. Then add the steering offset. The absolute encoder includes both positive and negative angles, so is the ticks are more than the half value of the maximum steering, we'll have negative angles:

  ```
  steer_angle = - (ticks_to_radians * (steer_max - steering_ticks)*2*pi/(steer_max)) + steer_offset
  ```
  Otherwhise we'll have positive angles:
  ```
  steer_angle = (ticks_to_radians * steering_ticks *pi *2 / (steer_max)) + steer_offset
  ```
  Given this the Predicted Uncalibrated Odometry of the front wheel is:

  <img src="https://github.com/CharlottePrimiceri/ProbabilisticRobotics/blob/main/04-Calibration/images/predicted_uncalibrated_odometry.png">

  Because there are negative angles we need to normalize the angles values when computing subtractions. First normalize the single angles within $-\pi$ and $\pi$ and then also the difference between each other:
  ```
  norm_theta_first = mod(theta_1 + pi, 2 * pi) - pi;
  ```
### Least Squares

- Apply the 