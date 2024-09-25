function delta_pose_T=predictFront_Tractor_Tricycle(traction_incremental_ticks,
                                                 steering_ticks, initial_state, kin_parameters, max_enc_values)
    
    steer_offset=kin_parameters(4);
    steer_max=max_enc_values(1);
    ticks_to_meters=kin_parameters(2);
    traction_max = max_enc_values(2);
    ticks_to_radians = kin_parameters(1);
    axis_lenght = kin_parameters(3);
    #i can find the steering angle but not the actual front displacement bc i don't have the radius
    #should i use the classic kinematic model by imposing some initial value on the front wheel? 
    #but i still can find the displacement by (current_tick - previous_tick)kt
    #i can put the computation for the overflow directly there
    
    #traction_incremental_ticks are the increments of the encoder computed in the dataset function
    #(ticks_to_meters / traction_max) is the value of meters corresponds to one single tick
    traction_front = traction_incremental_ticks * (ticks_to_meters / traction_max);
    
    # (ticks_to_radians *2*pi / steer_max) is the value of radians (converted from revolution to
    # radians with a factor 2pi)  corresponds to one single tick in case of positive angles
    if ticks_to_radians < (steer_max/2)
      steer_angle = steering_ticks * (ticks_to_radians *2*pi / steer_max) + steer_offset;
    else  
    # considering negative angles
      steer_angle = -steering_ticks * [(steer_max-ticks_to_radians) *2*pi / steer_max] + steer_offset;
    endif
    
    # drawing the model of the tricycle we obtain that relationship
    back_wheel_displacement = traction_front*cos(steer_angle);

    dth = back_wheel_displacement*sin(steer_angle)/axis_lenght;
    
    # need to find S and C
    S = [0.00000   0.00833   0.00000  -0.16667   0.00000   1.00000];
    C = [0.00139   0.00000  -0.04167   0.00000   0.50000   0.00000];

    dx = delta * polyval(S,dth);
    dy = delta * polyval(C,dth);

    delta_pose_T=[dx; dy; dth];


endfunction

function robot_config = robot_config_f(kin_parameters, initial_state, n, max_enc_values, u)
        inc_enc_column = u(:,3);
        abs_enc_column = u(:,2);
        Z=zeros(3, n);
        for i = 1:n
            traction_incremental_ticks = inc_enc_column(i,1);
            steering_ticks = abs_enc_column(i,1);
            Z(1:3,i) =predictFront_Tractor_Tricycle(traction_incremental_ticks,
                                                 steering_ticks, initial_state, kin_parameters, max_enc_values)
        endfor

endfunction