mp = motion_planner;

traj_num = 4;

%waypoints
waypoints(1, :) = [0.0, 0.0, 0.0];
waypoints(2, :) = [0.6, -0.6, 0.0];
waypoints(3, :) = [0.0, 0.6, 0.0];
waypoints(4, :) = [-0.6, -0.6, 0.0];
waypoints(5, :) = [0.0, 0.0, 0.0];

%trajectory flight times
traj_flight_times(1) = 1;
traj_flight_times(2) = 1;
traj_flight_times(3) = 1;
traj_flight_times(4) = 1;

mp = init(mp, waypoints, traj_flight_times, traj_num);
mp = plan_trajectory(mp);
plot_trajectories(mp);