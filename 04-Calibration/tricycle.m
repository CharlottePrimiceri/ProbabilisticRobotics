function delta_pose_T=predictFront_Tractor_Tricycle(traction_angular_offset,
                                                 measured_steering_angle, initial_state)
    
    #i can find the steering angle but not the actual front displacement bc i don't have the radius
    #should i use the classic kinematic model by imposing some initial value on the front wheel? 

    traction_front = ;
    steer_angle= ;
    delta = ;
    S = [];
    C = [];
    dx = delta * polyval(S,dth);
    dy = delta * polyval(C,dth);

    delta_pose_T=[dx; dy; dth];


endfunction
