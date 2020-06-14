classdef motion_planner    
    methods
        function A_end = generate_A_end_matrix(obj, t)
            A_end = [1 t t^2  t^3   t^4    t^5    t^6     t^7;
                     0 1 2*t  3*t^2 4*t^3  5*t^4  6*t^5   7*t^6;
                     0 0 2    6*t   12*t^2 20*t^3 30*t^4  42*t^5;
                     0 0 0    6     24*t   60*t^2 120*t^3 210*t^4];
        end
        
        function traj_polys = plan_optimized_segment(obj, start_val, end_val, flight_time)
            t = flight_time;
            
            %construct hessian matrix
            b0 = 24;
            b1 = 120;
            b2 = 360;
            b3 = 840;
            Q = zeros(8, 8);
            Q(5, 5) = b0 * b0 * t;
            Q(5, 6) = b0 * b1 * (1/2) * t^2;
            Q(5, 7) = b0 * b2 * (1/3) * t^3;
            Q(5, 8) = b0 * b3 * (1/4) * t^4;
            Q(6, 5) = b0 * b1 * (1/2) * t^2;
            Q(6, 6) = b1 * b1 * (1/3) * t^3;
            Q(6, 7) = b1 * b2 * (1/4) * t^4;
            Q(6, 8) = b1 * b3 * (1/5) * t^5;
            Q(7, 5) = b0 * b2 * (1/3) * t^3;
            Q(7, 6) = b1 * b2 * (1/4) * t^4;
            Q(7, 7) = b2 * b2 * (1/5) * t^5;
            Q(7, 8) = b2 * b3 * (1/6) * t^6;
            Q(8, 5) = b0 * b3 * (1/4) * t^4;
            Q(8, 6) = b1 * b3 * (1/5) * t^5;
            Q(8, 7) = b2 * b3 * (1/6) * t^6;
            Q(8, 8) = b3 * b3 * (1/7) * t^7;
            
            %construct equility constraints
            A_start = [1 0 0 0 0 0 0 0;
                       0 1 0 0 0 0 0 0;
                       0 0 2 0 0 0 0 0;
                       0 0 0 6 0 0 0 0];
              
            A_next = [0  0  0  0 0 0 0 0;
                      0 -1  0  0 0 0 0 0;
                      0  0 -2  0 0 0 0 0;
                      0  0  0 -6 0 0 0 0];
                 
            %generate A matrix
            wp_cnt = 2;      %TODO: replace with func arg
            t_list = [1; 1]; %TODO: replace with func arg
            A = zeros(wp_cnt * 4, (wp_cnt -1) * 8);
            size(A)
            for i = 1: (wp_cnt - 1)
                if i == 1
                    A_end = generate_A_end_matrix(obj, t_list(i));
                    
                    start_r = 1;
                    end_r = 1 + 4;
                    c = 1;                    
                    A(start_r:start_r+3, c:c+7) = A_start;
                    A(end_r:end_r+3, c:c+7) = A_end;
                else
                    A_end = generate_A_end_matrix(obj, t_list(i));
                    
                    next_r = (i*4) + 1;
                    end_r = (i+1)*4 + 1;
                    c = (i*4) + 1;
                    A(next_r:next_r+3, c:c+7) = A_next;
                    A(end_r:end_r+3, c:c+7) = A_end;
                end
            end
            %disp(A);
            
            d = [start_val; 0; 0; 0; end_val; 0; 0; 0];
            
            traj_polys = quadprog(Q, [], [], [], A, d);
        end
        
        function result=calc_7th_polynomial(obj, c, t)
            result = c(1) + ...
                     c(2)*t + ...
                     c(3)*t^2 + ...
                     c(4)*t^3 + ...
                     c(5)*t^4 + ...
                     c(6)*t^5 + ...
                     c(7)*t^6 + ...
                     c(8)*t^7;
        end
    end
end