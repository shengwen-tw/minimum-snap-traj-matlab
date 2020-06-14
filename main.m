ITERATION_TIMES = 1000;
mp = motion_planner;

%plot datas
time_arr = zeros(1, ITERATION_TIMES);
traj1_arr = zeros(1, ITERATION_TIMES);
traj2_arr = zeros(1, ITERATION_TIMES);

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

%plot trajectory 1
time_step = traj(1).t / ITERATION_TIMES;
for i = 1: ITERATION_TIMES
    time_arr(i) = i * time_step;
    traj_coeff = mp.get_traj_coeff_from_list(1, traj_coeff_list);
    traj1_arr(i) = mp.calc_7th_polynomial(traj_coeff, i * time_step);
end

%plot trajectory 2
time_step = traj(2).t / ITERATION_TIMES;
for i = 1: ITERATION_TIMES
    time_arr(i) = i * time_step;
    traj_coeff = mp.get_traj_coeff_from_list(2, traj_coeff_list);
    traj2_arr(i) = mp.calc_7th_polynomial(traj_coeff, i * time_step);
end

figure('Name', 'trajectory1');
plot(time_arr, traj1_arr);
xlabel('time [s]');
ylabel('x position [m]');

figure('Name', 'trajectory2');
plot(time_arr, traj2_arr);
xlabel('time [s]');
ylabel('x position [m]');