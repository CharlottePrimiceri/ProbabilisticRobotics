source "../tools/utilities/geometry_helpers_2d.m"

function A = v2t(v)
    % This function converts a vector to a transformation matrix
    A = [cos(v(3)), -sin(v(3)), v(1);
         sin(v(3)),  cos(v(3)), v(2);
         0,          0,        1];
end

function v = t2v(T)
    v = [T(1,3), T(2,3), atan2(T(2,1), T(1,1))];
end

#U: matrix contains odometry, it provides the incremental changes in the robot's position and orientation at each time step.

#defining function to compute trajectory T (so the absolute position and orientation)
function T= odometry_trajectory(U)
         
        T=zeros(size(U,1), 3);
        current_T=v2t(zeros(1,3));
        for i=1:size(U,1)
            u=U(i,1:3)';
            current_T *= v2t(u);
            T(i,1:3)=t2v(current_T)';
        end

end

#this function computes the error of the i^th measurement in Z
#given the calibration parameters
#i:	the number of the measurement
#X:	the actual calibration parameters
#Z:	the measurement matrix
#e:	the error of the ith measurement
function e=error_function(i,X,Z)
	uprime=Z(i,1:3)';
	u=Z(i,4:6)';
	e=uprime-X*u;
end

#derivative of the error function for the ith measurement in Z
#does not depend on the state
#i:	the measuement number
#Z:	the measurement matrix
#A:	the jacobian of the ith measurement
function A=jacobian(i,Z)
	u=Z(i,4:6);
	A=zeros(3,9);
	A(1,1:3)=-u;
	A(2,4:6)=-u;
	A(3,7:9)=-u;
end

#computes calibration matrix given the measurement matrix Z
function X=ls_calibrate_odometry(Z)
	
        H=zeros(9,9);
        b=zeros(9,1);
        #initial guess
        X=eye(3); 
        
        for i=1:size(Z,1),
            e=error_function(i,X,Z);
            A=jacobian(i,Z);
            H=H+A'*A;
            b=b+A'*e;
        end
        #linear system
        deltaX=-H\b;
        #this reshapes the 9x1 increment vector in a 3x3 matrix
        dX=reshape(deltaX,3,3)';
        #cumulative solution
        X=X+dX;
end

#applies correction to all odometries in U
function C=apply_odometry_correction(X, U)
        C=zeros(size(U,1),3);
        for i=1:size(U,1),
            u=U(i,1:3)';
            #given the bias parameters return unbiased odometry
            uc=X*u;
            C(i,:)=uc;
        end
end

# from least square x = x + deltax
# deltax is our perturbation
# x are our kinematic parameters to be estimated by minimizing the error of the laser pose prediction
function [e, J] = errorAndJacobian(x,U, kinematic_parameters, T, laser_baselink)
        meas = U(:, 7:9);
        pred = laser(kinematic_parameters, T, laser_baselink);
        pred_t = pred';
        e = pred - meas;


