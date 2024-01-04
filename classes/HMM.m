classdef HMM < handle
    %%
    properties
        Grid % grid class
        Robot % robot class
        A % transition matrix
        B % emission matrix
        pi % initial state
        t % current time point
        alpha % P(x,y) at current time, for all states x
        y % sensor reading idx at current time
        sensorbeep % list of sensors detecting obstacles at time t
    end
    %%
    methods
        %%
        % Class constructor
        function obj = HMM(simdata, Robot, direction_probs, matrix_pi)
            % Receives: Grid class, Robot class, vector of probabilities of
            % moving in each possible direction, initial coordinate
            % probabilities for each state
            
            obj.Grid = simdata;
            obj.Robot = Robot;
            
            obj.A = build_matrix_A(direction_probs, simdata.grid_size, simdata.obstacle_matrix, Robot.n_moves);
            obj.B = build_matrix_B(Robot.p_error, Robot.n_sensors, simdata.obstacle_matrix, simdata.grid_size);
            obj.pi = matrix_pi;
            obj.t = 0;
            obj.alpha = zeros(simdata.n_states, 1);
        end
        
        % Get a sensor reading
        function y = sensor_reading(obj)
            % Robot position
            robot_pos = obj.Robot.Robot_idx;
            % Sensor reading is argmax B(x); if there are multiple, choose
            % randomly
            maxval = max(obj.B(robot_pos,:));
            possible_ys = find(obj.B(robot_pos,:)==maxval);
            if length(possible_ys)==1
                y = possible_ys;
            else
                y = randsample(possible_ys, 1);
            end
            obj.y = y;
            % Find sensors that detected obstacles
            obj.sensorbeep = display_sensor(obj.y, obj.Robot.n_sensors);
        end
        
        % Forward recursion
        function prob_dist = forward_recursion(obj)
            % Calculate probability distribution of the robot on each
            % coordinate of the grid, and the new coordinate estimate for
            % its position
            
            % Update time
            obj.t = obj.t + 1;
            % Move robot
            obj.Robot.moveRobot;
            % Get new sensor reading
            sensor_reading(obj);
            
            if obj.t == 1
                % Calculate alphas
                for x = 1:obj.Grid.n_states
                    obj.alpha(x) = obj.B(x, obj.y) * obj.pi(x);
                end
                % Normalize alphas
                obj.alpha = obj.alpha/sum(obj.alpha);
                
            else
                % Calculate alphas
                new_alpha = obj.alpha;
                for x = 1:obj.Grid.n_states
                    new_alpha(x) = obj.B(x, obj.y) .* sum(obj.A(:, x) .* obj.alpha);
                end
                obj.alpha = new_alpha;
                % Normalize alphas
                obj.alpha = obj.alpha/sum(obj.alpha);
            end
                
            prob_dist = obj.alpha; % (already normalized <=> divided by Py)                
            prob_dist = reshape(prob_dist, obj.Grid.grid_size);
            prob_dist = flipud(prob_dist'); % swap x and y
            
        end
                    
    end

end
    
    