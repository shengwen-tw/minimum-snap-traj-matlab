mp = motion_planner;

p_start = [0; 0; 0];
p_end = [1, 2, 3];

mp.plan_optimized_segment(p_start(1), p_end(1), 10)