%show the uncalibrated and calibrated odometry? 
close all
clear
clc
#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
#source "../04-Calibration/odometry_trajectory.m"
#source "../04-Calibration/tricycle.m"
disp('loading the matrix');
path = 'dataset.txt';
# Read dataset.... transform then into function

#initial guess [x y theta psi]
initial_state = [0; 0; 0];


function odometry=read_odometry(path)
        file = fopen(path, 'r');

        if file == -1
            error('Failed to open the file.');
        end

        #reads text into a cell array of strings, each line is a cell
        dataset = textscan(file, '%s', 'Delimiter', '\n', 'MultipleDelimsAsOne', 1);
        lines = dataset{1};
        n_lines = size(lines)(1);
        #disp(n_lines);
        overflow_inc_enc = 4294967295;
        previous_inc_t = 4294859756;
        inc_enc_update = 0;
        #odometry matrix, without first 8 lines
        odometry = zeros(n_lines-8, 9);
        #from 9 so it doesn't include the first 8 rows
        for i = 9:n_lines
            line = lines{i};
            line = textscan(line, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
            tick_inc = line{3};
            current_inc_t = tick_inc;
            delta = current_inc_t - previous_inc_t;
            overflow_delta = overflow_inc_enc + delta;
            # if those two conditions occurs there is overflow
            if (current_inc_t < previous_inc_t && overflow_delta < (previous_inc_t - current_inc_t)) #delta < (overflow_inc_enc/2))
               inc_enc_update = overflow_delta;
               tick_inc = inc_enc_update;
               #display('overflow')
            else
               inc_enc_update = delta;
               tick_inc = inc_enc_update;
               #display('non overflow')
            endif
            previous_inc_t = current_inc_t;
            #display(tick_inc) 
            #line{3} = tick_inc;
            #odometry(i-8,:) = cell2mat(line(1:9)); #here modified 4:6
            odometry(i-8,:)=[cell2mat(line(1:2)), tick_inc, cell2mat(line(4:9))];
        endfor    
endfunction

U = read_odometry(path);
#display(U(:,3))

function new_dataset=eliminate_overflow(U)

        # because of the floating point, is better to reinitialize the time to have more precise increment value of time
        #in matlab if we compute the eps(number_a), the error that can be computed between number_a and the minum computable consecutive one number_b,
        #eps(1.6e+09) = 2.38e-07 
        #if there is an increment of the last two digits in 1668091584.821040869 (example of our dataset) it would be lost.
        time_column = U(:,1);
        time_update = 0;
        previous_time = time_column(1,1);
        n =size(time_column,1);
        time_column(1,1) = time_update;
        for i = 1:(n-1)
            current_time = time_column(i+1,1);
            time_increment = current_time - previous_time;
            time_update += time_increment;
            time_column(i+1,1) = time_update;
            previous_time = current_time;
        endfor

        #the incremental encoder values are stored in a uint32 variable with max range of:
        overflow_inc_enc = 4294967295;

        #if the previous value of tick is greater than the next one but cannot be considered 
        #as a backward motionthere is overflow, so write the change in
        #ticks as joints_max_enc_values - previous_tick_value + new_tick_value
        #otherwhise maintain the actual difference
        inc_enc_column = U(:,3);
        inc_enc_update = 0;
        previous_inc_t = inc_enc_column(1,1);
        for i = 1:(n-1)
            current_inc_t = inc_enc_column(i+1,1);
            delta = current_inc_t - previous_inc_t;
            # if those two conditions occurs there is overflow
            if current_inc_t < previous_inc_t && delta > (overflow_inc_enc/2)
               inc_enc_update = overflow_inc_enc - delta;
               inc_enc_column(i+1,1) = inc_enc_update;
            else
                inc_enc_update = delta;
                inc_enc_column(i+1,1) = inc_enc_update;
            endif
            previous_inc_t = current_inc_t;    
        endfor
        U(:,1)= time_column;
        U(:,3) = inc_enc_column;
        new_dataset = U;
endfunction


#U = read_odometry(path);
#model_pose = U(:,4:6);
#display(U(56:60,3))
display('finisce di far vedere la colonna di tick')

overflow_inc_enc = 4294967295;
inc_enc_column_pre = U(:,3);
inc_enc_update = 0;
n = size(inc_enc_column_pre,1);
previous_inc_t = inc_enc_column_pre(1,1);
for i = 1:(35)
    current_inc_t = inc_enc_column_pre(i,1);
    delta = current_inc_t - previous_inc_t; 
    # if those two conditions occurs there is overflow
    if current_inc_t < previous_inc_t && delta > (overflow_inc_enc/2)
        inc_enc_update = overflow_inc_enc - delta;
        inc_enc_column(i,1) = inc_enc_update;
    else
        inc_enc_update = delta;
        inc_enc_column(i,1) = inc_enc_update;
    endif
previous_inc_t = current_inc_t;    
endfor

#display(inc_enc_column(35,1))

#u_new = eliminate_overflow(U);
#inc = u_new(:,3);
#display(inc(1,1))
#display(u_new(:,5))
#plot ground truth of the laser pose trajectory 
#plot(model_pose(:,1),model_pose(:,2),-'o' ); 
# chosen those value of axis just beacuse i've known the true trajectory from the python file provided
#axis([-5 4 -4 2]);
#pause(10);

initial_kin_parameters = [0.1 0.0106141 1.4 0 1.5 0 0];
max_enc_values = [8192 5000];
#n = size(u_new(:,2),1);

#T = robot_config_f(initial_state, max_enc_values, u_new);
#display(T)
#initial_kin_parameters(1)
#disp(inc_enc_column);
%disp(size(time_column,1))
#compute the ground truth trajectory
#T = odometry_trajectory(U(:,4:6));
#disp('ground truth');
#hold on;
#'r-', 'linewidth', 2);
#compute the uncalibrated odometry
#uncalibrated_odometry = odometry_trajectory(U(:,1:3));
#disp('Uncalibrated Odometry');
#hold on;
#plot(uncalibrated_odometry(:,1), uncalibrated_odometry(:,2), 'b-', 'linewidth', 2);
#pause(10);


%disp('2D position of sensor w.r.t. base link');
%S = function1();
%disp(S);
%pause(1);

%disp('kinematic parameters');
%P = function2();
%disp(P);
steer_offset=initial_kin_parameters(4);
steer_max=max_enc_values(1);
ticks_to_meters=initial_kin_parameters(2);
traction_max = max_enc_values(2);
radians_one_tick = initial_kin_parameters(1);
axis_lenght = initial_kin_parameters(3);
inc_enc_column = U(:,3);
abs_enc_column = U(:,2);
#display(inc_enc_column)
display('oooooooooooooooo')
#display(inc_enc_column(1,1))
#traction_incremental_ticks = inc_enc_column(68,1);
traction_incremental_ticks = 10902;
#steering_ticks = abs_enc_column(68,1);
steering_ticks = 8012;
traction_front = traction_incremental_ticks * (ticks_to_meters / (traction_max)); #0,00111660332
display(traction_front)

if steering_ticks < (steer_max/2)
   steer_angle = steering_ticks * (radians_one_tick *2*pi / steer_max) + steer_offset; #0,0222314453
   display('positive angle')
else  
# considering negative angles
   steer_angle = -radians_one_tick  * [(steer_max-steering_ticks) *2*pi / steer_max] + steer_offset;
   display('negative angle')
endif
state = [1.99646 0.0978532  0.109438];
theta = state(3);
display(steer_angle)
dx = traction_front *cos(theta + steer_angle);
dy = traction_front *sin(theta + steer_angle);
dth = (traction_front/axis_lenght) * sin(steer_angle);

#back_wheel_displacement = traction_front*cos(steer_angle); #0,00108911
#display(back_wheel_displacement)
#dth = traction_front*sin(steer_angle)/axis_lenght;
#S = [-1/5040  0     1/120 0     -1/6  0   1];
#C = [0        1/720 0     -1/24 0     1/2 0];
#dx = back_wheel_displacement * polyval(S,dth);
#dy = back_wheel_displacement * polyval(C,dth);
display(dx)
display(dy)
display(dth)
x = state(1) + dx;
y = state(2) + dy;
th = state(3) + dth;
pose_T = [x; y; th];
display(pose_T)
#display(inc_enc_column(27,1))

function pose_T=predictFront_Tractor_Tricycle(traction_incremental_ticks,
                                                 initial_state, steering_ticks, max_enc_values)
        kin_parameters = [0.1 0.0106141 1.4 0 1.5 0 0];
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
        if steering_ticks < (steer_max/2)
           steer_angle = steering_ticks * (ticks_to_radians *2*pi / steer_max) + steer_offset;
        else  
        # considering negative angles
           steer_angle = -steering_ticks * [(steer_max-ticks_to_radians) *2*pi / steer_max] + steer_offset;
        endif
        
        # drawing the model of the tricycle we obtain that relationship
        #back_wheel_displacement = traction_front*cos(steer_angle);

        #dth = traction_front*sin(steer_angle)/axis_lenght;
        
        # need to find S and C
        #S = [-1/5040  0     1/120 0     -1/6  0   1];
        #C = [0        1/720 0     -1/24 0     1/2 0];


        #dx = back_wheel_displacement * (sin(dth)/dth);
        #dy = back_wheel_displacement * (1-cos(dth))/dth;
        
        #dx = back_wheel_displacement * polyval(S,dth);
        #dy = back_wheel_displacement * polyval(C,dth);
        #pose_T=[dx; dy; dth];
        theta = initial_state(3);
        display(steer_angle)
        dx = traction_front *cos(theta + steer_angle);
        dy = traction_front *sin(theta + steer_angle);
        dth = (traction_front/axis_lenght) * sin(steer_angle);
        x = initial_state(1) + dx;
        y = initial_state(2) + dy;
        th = initial_state(3) + dth;
        pose_T = [x; y; th];

endfunction

function Z = robot_config_f(initial_state, max_enc_values, u_new)
        inc_enc_column = u_new(:,3);
        abs_enc_column = u_new(:,2);
        n = size(inc_enc_column, 1);
        Z=zeros(3, n);
        for i = 1:n
            traction_incremental_ticks = inc_enc_column(i);
            steering_ticks = abs_enc_column(i);
            Z(1:3,i) = predictFront_Tractor_Tricycle(traction_incremental_ticks,
                                                 initial_state, steering_ticks, max_enc_values)
            initial_state = Z(1:3,i);
        endfor
        
endfunction
display(inc_enc_column(5))
display(size(inc_enc_column, 1))

#inc_enc_column = u_new(:,3);
#display(inc_enc_column)
#abs_enc_column = u_new(:,2);
#n = size(abs_enc_column, 1);
#Z=zeros(3, n);
#delta_pose_T=[3; 4; 5];
#Z(1:3,2) = delta_pose_T; 
#display(delta_pose_T(3))
#display(delta_pose_T)
#U(1:71, :) = [];
T = robot_config_f(initial_state, max_enc_values, U);
#display(T(1:3,1:691))
#display(T(1:3,35))
#display(T(1:3,36))
#display(T(1:3,37))
#display(T(2,:))
plot(T(1,:),T(2,:),-'o'); 
# chosen those value of axis just beacuse i've known the true trajectory from the python file provided
axis([-5 25 -13 2]);
pause(10);