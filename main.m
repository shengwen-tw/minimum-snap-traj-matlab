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
traj_size = 2;

%trajectory planning
traj_coeff_list=mp.plan_optimized_segment(traj, traj_size);
traj1_coeff = mp.get_traj_coeff_from_list(1, traj_coeff_list);
traj2_coeff = mp.get_traj_coeff_from_list(2, traj_coeff_list);

PLOT_TIMES_PER_SECOND = 50;
ITERATION_TIMES = (traj(1).t + traj(2).t) * PLOT_TIMES_PER_SECOND;

%plot arrays
time_arr = zeros(1, ITERATION_TIMES);
traj_arr = zeros(1, ITERATION_TIMES);

%plot trajectory 1
elapsed_plot_time = 0;
traj_plot_times = traj(1).t * PLOT_TIMES_PER_SECOND;
time_step = traj(1).t / traj_plot_times;
for i = 1: traj_plot_times
    traj_arr(i) = mp.calc_7th_polynomial(traj1_coeff, (i-1) * time_step);
end

%plot trajectory 2
elapsed_plot_time = traj_plot_times;
traj_plot_times = traj(2).t * PLOT_TIMES_PER_SECOND;
time_step = traj(2).t / traj_plot_times;
for i = 1: traj_plot_times
    traj_arr(elapsed_plot_time + i) = mp.calc_7th_polynomial(traj2_coeff, (i-1) * time_step);
end

for i = 1: ITERATION_TIMES
        time_arr(i) = (i-1) * time_step;
end

figure('Name', 'trajectory');
plot(time_arr, traj_arr);
xlabel('time [s]');
ylabel('x position [m]');
