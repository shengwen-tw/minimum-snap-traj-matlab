ITERATION_TIMES = 1000;
mp = motion_planner;

%plot datas
time_arr = zeros(1, ITERATION_TIMES);
traj_x_arr = zeros(1, ITERATION_TIMES);

%waypoints
p_start = [0.5; 0; 0];
p_end = [1, 2, 3];

%trajectory planning
traj1_flight_time = 1;
traj1_coeff=mp.plan_optimized_segment(p_start(1), p_end(1), traj1_flight_time);

%plot trajectory 1
time_step = traj1_flight_time / ITERATION_TIMES;
for i = 1: ITERATION_TIMES
    time_arr(i) = i * time_step;
    traj_x_arr(i) = mp.calc_7th_polynomial(traj1_coeff, i * time_step);
end

figure('Name', 'trajectory');
plot(time_arr, traj_x_arr);
xlabel('time [s]');
ylabel('x position [m]');