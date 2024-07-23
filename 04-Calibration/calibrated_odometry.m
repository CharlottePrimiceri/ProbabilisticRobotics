#U: matrix contains odometry 

#defining function to compute trajectory
function T= odometry_trajectory()
         
         T=zeros(size(U,1), 3);
         current_T=v2t(zeros(1,3));
         for i=1:size(U,1),
            

endfunction




function delta_pose_T=predictFront_Tractor_Tricycle(traction_angular_offset,
                                                 measured_steering_angle, x)
    tr = 
    tl =
    dx = 
    dy = 
    dth = 
    delta_pose_T=[dx; dy; dth]

endfunction