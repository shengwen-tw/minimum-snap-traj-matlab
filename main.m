ITERATION_TIMES = 1000;
mp = motion_planner;

%plot datas
time_arr = zeros(1, ITERATION_TIMES);
traj_x_arr = zeros(1, ITERATION_TIMES);

traj(1).p_start = 0.5;
traj(1).p_end = 1.0;
traj(1).t = 1.0;

traj(2).p_start = 1.0;
traj(2).p_end = 2.0;
traj(2).t = 1.0;

traj_size = 1;

%trajectory planning
p_start = 0.5;
p_end = 1.0;
t = 1;
traj1_coeff=mp.plan_optimized_segment(traj, 1);

%plot trajectory 1
time_step = traj(2).t / ITERATION_TIMES;
for i = 1: ITERATION_TIMES
    time_arr(i) = i * time_step;
    traj_x_arr(i) = mp.calc_7th_polynomial(traj1_coeff, i * time_step);
end

figure('Name', 'trajectory');
plot(time_arr, traj_x_arr);
xlabel('time [s]');
ylabel('x position [m]');