#U: matrix contains odometry, itprovides the incremental changes in the robot's position and orientation at each time step.

#defining function to compute trajectory T (so the absolute position and orientation)
function T= odometry_trajectory()
         
         T=zeros(size(U,1), 3);
         current_T=v2t(zeros(1,3));
         for i=1:size(U,1),
                u=U(i,1:3)';
                current_T*=v2t(u);
		        T(i,1:3)=t2v(current_T)';
	     end

endfunction

function C=apply_odometry_correction(X, U)
	C=zeros(size(U,1),3);
	for i=1:size(U,1),
		u=U(i,1:3)';
		uc=;
		C(i,:)=uc;
	end
end
