classdef Robot <handle
    %%
    properties
        Grid % Grid class where the robot is positioned
        Robot_coords % Coordinates of the robot in the grid
        Robot_idx % idx of the robot in the grid
        n_sensors % number of sensors of the robot
        n_moves % number of directions the robot can move to
        p_error % list with error probability for each robot sensor
    end
    %%
    methods
        % Class constructor
        function obj = Robot(simdata, n_sensors, n_moves, p_error)
            obj.Grid = simdata;
            
            % Robot starting point : random coord ========================
            % Get all possible ids for starting positions
            possible_coords = setdiff(1:simdata.n_states, simdata.obstacles_ids);
            
            % Randomly choose a starting id for the robot
            obj.Robot_idx = possible_coords(randi(length(possible_coords),1));
            obj.Robot_coords = coords_from_idx(obj.Robot_idx, simdata.grid_size);
            
            % Robot coords are (d,s), where s=1 is the southernmost point
            % and s = number of lines is the northernmost point. d=0 is the
            % east, d = number of columns is west

            obj.n_sensors = n_sensors;
            obj.n_moves = n_moves;
            obj.p_error = p_error;
        end
        
        % Move robot
        function moveRobot(obj)
            % Find all the possible directions for the robot to move
            % according to
            if obj.n_moves == 4
                %possible_dirs = ['N', 'S', 'W', 'E', 'X'];
                possible_dirs = [1, 2, 3, 4, 9];
            elseif obj.n_moves == 8
                %possible_dirs = ['N', 'S', 'W', 'E', 'NW', 'NE', 'SW', 'SE', 'X'];
                possible_dirs = [1, 2, 3, 4, 5, 6, 7, 8, 9];
            end
            % Figure out which neighbours have obstacles
            obstacles_ids = check_near_obstacles(obj.Robot_coords, obj.Grid.obstacle_matrix, obj.Grid.grid_size);
            % Remove directions that would lead to an obstacle
            possible_dirs = setdiff(possible_dirs, obstacles_ids);
            
            % Pick a random direction
            direction = possible_dirs(randi(length(possible_dirs),1));
            
            if direction == 9 %'X', stay in same position
                destination = obj.Robot_coords;
            elseif direction == 1% 'N', s increases (top)
                destination = obj.Robot_coords + [0,1];
            elseif direction == 2 % 'S', s decreases (bottom)
                destination = obj.Robot_coords + [0,-1];
            elseif direction == 3 %'W', d decreases (left)
                destination = obj.Robot_coords + [-1,0];
            elseif direction == 4 %'E', d increases (right)
                destination = obj.Robot_coords + [1,0];
            elseif direction == 5 %'NW'
                destination = obj.Robot_coords + [-1,1];
            elseif direction == 6 %'NE'
                destination = obj.Robot_coords + [1, 1];
            elseif direction == 7 %'SW'
                destination = obj.Robot_coords + [-1, -1];
            elseif direction == 8 %'SE'
                destination = obj.Robot_coords + [1, -1];
            else
                error("Direction not available.")
            end
            % Update robot position
            obj.Robot_coords = destination;
            obj.Robot_idx = idx_from_coords(destination, obj.Grid.grid_size);
        end
        
        % Relocate robot
        function relocate_object(obj)
            % Pick robot and place it on another random location;
            
            % Get all possible ids for positions
            possible_coords = setdiff(1:simdata.n_states, simdata.obstacles_ids);
            % Randomly choose an id for the robot
            obj.Robot_idx = possible_coords(randi(length(possible_coords),1));
            obj.Robot_coords = coords_from_idx(obj.Robot_idx, simdata.grid_size);
            
        end
    end
end