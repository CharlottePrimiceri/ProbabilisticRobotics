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
  
  rotation: [0, 0, 0, 1]
  
For a certain time step we have:
- ticks of the absolute encoder for the steering, with maximum value for a joint of 8192;
- tick of the incremental encoder for the traction, with maximum value for a joint of 5000;
- model pose;
- tracker pose of the sensor. 


## Task

Find the output:
- 2D position of the sensor w.r.t the base link
- The kinematic parameters [ ksteer, ktraction, steer_offset, base_line ]
- Reach the correct trajectory:

![Figure_1](https://github.com/user-attachments/assets/ca34aed1-7f20-48aa-b1fd-7660e9ba3524)

## Solution