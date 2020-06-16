mp = motion_planner;

%trajectory parameters
traj(1).p_start = 0.5;
traj(1).p_end = 1.0;
traj(1).t = 1.0;
%
traj(2).p_start = 1.0;
traj(2).p_end = 2.0;
traj(2).t = 1.0;
%
traj(3).p_start = 2.0;
traj(3).p_end = 4.0;
traj(3).t = 1.0;
%
traj_size = 3;

%trajectory planning
traj_coeff_list=mp.plan_optimized_segment(traj, traj_size);

total_flight_time = 0;
for i = 1: traj_size
    %create coefficient list for all trajectories
    traj_coeff(i, :) = mp.get_traj_coeff_from_list(i, traj_coeff_list);
    
    %calculate total flight time of all trajectories
    total_flight_time = total_flight_time + traj(i).t;
end

%create arrays for plotting
PLOT_TIMES_PER_SECOND = 50;
ITERATION_TIMES = total_flight_time * PLOT_TIMES_PER_SECOND;
time_arr = zeros(1, ITERATION_TIMES);
traj_arr = zeros(1, ITERATION_TIMES);

%plot trajectories
elapsed_index = 0;
for i = 1: traj_size
    %calculate i-th trajectory plot times and time step
    traj_plot_times = traj(i).t * PLOT_TIMES_PER_SECOND;
    time_step = traj(i).t / traj_plot_times;
    
    %plot i-th trajectory
    for j = 1: traj_plot_times
        %update trajectory array
        traj_arr(elapsed_index + j) = mp.calc_7th_polynomial(traj_coeff(i, :), (j-1) * time_step);
        
        %update elapsed time array
        time_arr(elapsed_index + j) = (elapsed_index + j - 1) * time_step;
    end
    
    %accumlate total elapsed time
    elapsed_index = elapsed_index + traj_plot_times;
end

figure('Name', 'trajectory');
plot(time_arr, traj_arr);
xlabel('time [s]');
ylabel('x position [m]');
pause;
close all;
