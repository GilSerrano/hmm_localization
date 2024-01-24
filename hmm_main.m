function hmm_main()
    % Add folder and all subfolders to the path.
    folder = fileparts(which('hmm_main.m')); 
    addpath(genpath(folder));
    
    simdata = struct('running', 0, 'mode', 0, 'cnt', 0, 'prevCoord', [1, 1], 'agentisPlotted', false);
    
    getGrid(); % Receive user inputs
    
    plotInputGrid(); % Plot environment

    plotOutputGrid(zeros(simdata.N)); % Plot estimated probability distribution
    
    plotSensorGrid([]); % Plot sensors that are detecting obstacles

    mainLoop();
    
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    function mainLoop()
        iter = 0;
        cnt = 0;
        
        uiwait(); % Wait until start button is clicked
        
        simdata.grid_size = [simdata.N, simdata.N];
        simdata.n_states = simdata.N*simdata.N;
        
        % Calculate obstacle ids (list)
        simdata.obstacle_matrix = flipud(simdata.gridData); % flip matrix vertically so that it matches display coordinates
        simdata.obstacles_ids = find(simdata.obstacle_matrix'==1); % get indices where obstacles are
        simdata.n_obstacles = length(simdata.obstacles_ids);
        
        % Convert obstacle indices into display coordinates
        simdata.Obstacles_coords = {};
        for i = 1:simdata.n_obstacles
            simdata.Obstacles_coords{i} = coords_from_idx(simdata.obstacles_ids(i), simdata.grid_size)';
        end    

        % Create base HMM class
        %pi = ones(simdata.grid_size(1)*simdata.grid_size(2), 1)./(simdata.grid_size(1)*simdata.grid_size(2)); % pi is a uniform distribution
        pi0 = calculate_pi0(simdata);
        % pi0_matrix = flipud(matrix_from_pi0(pi0, simdata.N)');
        pi0_matrix = matrix_from_pi0(pi0, simdata.N)';
        plotOutputGrid(pi0_matrix);
        robot = Robot(simdata, simdata.n_sensors, simdata.n_moves, simdata.err_prob);
        simdata.HMMclass = HMM(simdata, robot, simdata.moving_probs, pi0);


        while simdata.running ~= 0 && iter < simdata.maxT
    
            if simdata.mode == 0 % stop

            elseif simdata.mode == 1 % one step at a time
                while cnt < simdata.cnt
                    probMap = simdata.HMMclass.forward_recursion();
                    plotOutputGrid(probMap);
                    plotAgent();
                    plotSensorGrid(simdata.HMMclass.sensorbeep);
                    iter = iter + 1;
                    cnt = cnt + 1;
                end

            elseif simdata.mode == 2 % running
                probMap = simdata.HMMclass.forward_recursion();
                plotOutputGrid(probMap);
                plotAgent();
                plotSensorGrid(simdata.HMMclass.sensorbeep);
                iter = iter + 1;           
            end      
            
            sgtitle(sprintf('Iteration %d', iter));
%             % Set the figure renderer to painters for color EPS
%             set(gcf, 'Renderer', 'painters');
%             file_name = sprintf('imgs/img_iter_%01d.eps', iter);  % '02' adds leading zeros if necessary
%             % Save the plot to EPS format
%             saveas(gcf, file_name, 'epsc');
%             file_name = sprintf('imgs/img_iter_%01d.png', iter);  % '02' adds leading zeros if necessary
%             % Save the plot to EPS format
%             saveas(gcf, file_name, 'png');
            pause(0.5);
        end
    
    end
    
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    % Function to get user input
    function getGrid(~,~)
        % Load information
        user_input = inputdlg({'Enter the size N of the grid map:',...
            'Enter the maximum number of iterations:',...
            'Enter the number of directions the robot can move to:'...
            'Enter the number of sensors the robot has:'...
            }, 'User input', [1 60; 1 60; 1 60; 1 60], {'10', '50', '8', '8'});
        N = user_input{1};
        maxT = user_input{2};
        n_moves = user_input{3};
        n_sensors = user_input{4};
        
        N = str2double(N);
        % Check if N is a positive integer
        if ~isnumeric(N) || N <= 0 || rem(N, 1) ~= 0
            error('N must be a positive integer');
        end
        
        maxT = str2double(maxT);
        % Check if maxT is a positive integer
        if ~isnumeric(maxT) || maxT <= 0 || rem(maxT, 1) ~= 0
            error('Maximum number of iterations must be a positive integer');
        end
        
        n_moves = str2num(n_moves);
        % Check if n_moves is 4 or 8
        if n_moves ~=4 && n_moves ~=8
            error('Only directions = 4 or directions = 8 are supported');
        end
        
        n_sensors = str2num(n_sensors);
        % Check if n_sensors is 4 or 8
        if n_sensors ~=4 && n_sensors ~=8
            error('Only 4 or 8 sensors are supported');
        end
        
        sensorerr = inputdlg({'N:', 'S:', 'W:', 'E:', 'NW:', 'NE:', 'SW:', 'SE:'}, 'Sensor error probability', [1 60; 1 60; 1 60; 1 60; 1 60; 1 60; 1 60; 1 60], {'0', '0', '0', '0', '0', '0', '0', '0'});
        err_prob = zeros(n_sensors, 1);
        for i=1:n_sensors
            err_prob(i,1) = str2double(sensorerr{i});
        end
        
        if n_moves ==8
            moves = inputdlg({'N:', 'S:', 'W:', 'E:', 'NW:', 'NE:', 'SW:', 'SE:', 'X:'}, 'Moving probability', [1 60; 1 60; 1 60; 1 60; 1 60; 1 60; 1 60; 1 60; 1 60], {'1/9', '1/9', '1/9', '1/9', '1/9', '1/9', '1/9', '1/9', '1/9'});
            moving_prob = zeros(9, 1);
            for i=1:9
                moving_prob(i,1) = sym(moves{i});
            end
        elseif n_moves ==4
            moves = inputdlg({'N:', 'S:', 'W:', 'E:', 'X:'}, 'Moving probability', [1 60; 1 60; 1 60; 1 60; 1 60], {'1/5', '1/5', '1/5', '1/5', '1/5'});
            moving_prob = zeros(5, 1);
            for i=1:5
                moving_prob(i,1) = sym(moves{i});
            end
        end
        
        if abs(sum(moving_prob)-1) > 1e-12
            error("Probabilities for each movement direction must sum to 1.")
        end
        
        simdata.N = N;
        simdata.maxT = maxT;
        simdata.n_moves = n_moves;
        simdata.n_sensors = n_sensors;
        simdata.err_prob = err_prob;
        simdata.moving_probs = moving_prob;

        % Create an empty gridData matrix initially filled with zeros
        simdata.gridData = get_default_grid(N);
        
        
        f = figure(); hold on;
        f.Position = [000 000 1300 600];

        simdata.f = f;
        simdata.s1 = subplot(1,5,1:2);
        simdata.s2 = subplot(1,5,3:4);
        simdata.s3 = subplot(1,5,5);

        % fprintf('End of getGrid.\n');
            
    end


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    function plotInputGrid(~,~)

        N = simdata.N;

        subplot(simdata.s1);
        % Create x and y coordinates for the grid
        x = linspace(0, N, N+1);
        y = linspace(0, N, N+1);
        
        hold off;
        % Plot the initial blank grid
        for i = 1:N+1
            plot([x(i), x(i)], [0, N], 'k');  % Vertical lines
            hold on;
            plot([0, N], [y(i), y(i)], 'k');  % Horizontal lines
        end
        
        simdata.obstacle_plots_1 = cell(N,N);
        simdata.obstacle_plots_2 = cell(N,N);
        
        for x_coord = 1:N
            for y_coord = 1:N
                if simdata.gridData(y_coord, x_coord) == 1
                    % Add obstacle to the clicked square (automatically
                    % resized to cell size)
                    rh1 = arrayfun(@(i) line([x_coord-1,x_coord], [y_coord-1, y_coord], 'Color', 'black', 'LineWidth',2), 1:N); 
                    rh2 = arrayfun(@(i) line([x_coord-1,x_coord], [y_coord, y_coord-1], 'Color', 'black', 'LineWidth',2), 1:N); 
                    simdata.obstacle_plots_1{y_coord, x_coord} = rh1;
                    simdata.obstacle_plots_2{y_coord, x_coord} = rh2;
                    
                end
            end
        end
        
        % Set plot properties
        axis square;
        xlim([0, N]);
        xticks([0.5:N-0.5]); xtickformat('%.0f')
        ylim([0, N]);
        yticks([0.5:N-0.5]); ytickformat('%.0f')
        xlabel('X-axis');
        ylabel('Y-axis');
        title(sprintf('%d-by-%d Grid (Click to Add Obstacles)', N, N));

        % Allow the user to click on squares to toggle obstacles
        set(gca, 'ButtonDownFcn', @toggleObstacle);

        % Create UI elements
        startButton = uicontrol('Style', 'pushbutton', 'String', 'Start', 'Position', [100, 50, 100, 30], 'Callback', @startCallback);
        agentButton = uicontrol('Style', 'pushbutton', 'String', 'Initial Position', 'Position', [200, 50, 200, 30], 'Callback', @agentCallback);
        oneStepButton = uicontrol('Style', 'pushbutton', 'String', 'One step', 'Position', [400, 50, 100, 30], 'Callback', @oneStepCallback);
        runButton = uicontrol('Style', 'pushbutton', 'String', 'Run', 'Position', [500, 50, 100, 30], 'Callback', @runCallback);
        stopButton = uicontrol('Style', 'pushbutton', 'String', 'Stop', 'Position', [600, 50, 100, 30], 'Callback', @stopCallback);
        % clearButton = uicontrol('Style', 'pushbutton', 'String', 'Clear', 'Position', [600, 50, 100, 30], 'Callback', @clearCallback);
        endButton = uicontrol('Style', 'pushbutton', 'String', 'End', 'Position', [700, 50, 100, 30], 'Callback', @endCallback);
        hold off;
        % fprintf('End of plotGrid.\n');
    end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    function plotOutputGrid(probabilityMatrix)
        
        subplot(simdata.s2);
        simdata.output_plot = imagesc(probabilityMatrix);
        customColormap = hot;
%         customColormap(1, :) = [0, 0, 1]; % Set the first color to blue for 0 probability
        
        colormap(simdata.s2, customColormap); % Apply the custom colormap
        colorbar; % Add a colorbar to show the mapping of values to colors
        lim_sup = min(1, max(probabilityMatrix, [], 'all')+0.001);
        caxis([0, lim_sup]);
        
        hold off; % Release the hold on the plot
        set(gca,'YDir','normal'); % Plot image with Y axis normal - not upside down
        
        % Adjusting the axes and setting labels
        axis square;
        xlabel('X-axis');
        ylabel('Y-axis');
        N = simdata.N;
        xticks([1:N]); xtickformat('%.0f')
        yticks([1:N]); ytickformat('%.0f')
        title('Heatmap-like Visualization of Probabilities');
        
        % Resize plot to same size as s1, regardless of colorbar
        s1Pos = get(simdata.s1,'position');
        s2Pos = get(simdata.s2,'position');
        s2Pos(3:4) = [s1Pos(3:4)];
        set(simdata.s2,'position',s2Pos);
        
    end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function plotSensorGrid(sensor_vec)

    subplot(simdata.s3);
    sensor_matrix = zeros(3,3);
    sensor_matrix(2,2) = -1;
    
    % Set sensor matrix = 1 where sensor is detecting an obstacle
    % Set sensor matrix = -1 where there are no sensors
    if ismember(1, sensor_vec)
        sensor_matrix(1, 2) = 1;
    end
    if ismember(2, sensor_vec)
        sensor_matrix(3, 2) = 1;
    end
    if ismember(3, sensor_vec)
        sensor_matrix(2, 1) = 1;
    end
    if ismember(4, sensor_vec)
        sensor_matrix(2, 3) = 1;
    end
    
    if simdata.n_sensors ==8
        if ismember(5, sensor_vec)
            sensor_matrix(1, 1) = 1;
        end
        if ismember(6, sensor_vec)
            sensor_matrix(1, 3) = 1;
        end
        if ismember(7, sensor_vec)
            sensor_matrix(3, 1) = 1;
        end
        if ismember(8, sensor_vec)
            sensor_matrix(3, 3) = 1;
        end
    else
        sensor_matrix(1,1) = -1;
        sensor_matrix(1,3) = -1;
        sensor_matrix(3,1) = -1;
        sensor_matrix(3,3) = -1;
    end
    
    imagesc(sensor_matrix, [-1,1]);
    disp(sensor_matrix);
    
    colour0 = [0 0 0]; % black for no sensors
    colour1 = [1 1 1]; % white for sensors not detecting obstacles
    colour2 = [1 0 0]; % red for sensors detecting obstacles
    colormap(simdata.s3, [colour0; colour1; colour2]);
    
    % Create x and y coordinates for the grid
    x = [1.5, 2.5];
    y = [1.5, 2.5];

    hold on
    % Plot grid
    for i = 1:2
        plot([x(i), x(i)], [0, 3.5], 'k');  % Vertical lines
        plot([0, 3.5], [y(i), y(i)], 'k');  % Horizontal lines
    end
    hold off

    % Adjusting the axes and setting labels
    axis square;
    title('Sensor Readout');
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
   
end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    function plotAgent()
        
        subplot(simdata.s1)
        if simdata.agentisPlotted
            delete(simdata.agentPlot);
        end
        
        nextCoord = simdata.HMMclass.Robot.Robot_coords; 
        th = 0:pi/50:2*pi;
        xunit = 0.4*cos(th) + nextCoord(1)-0.5;
        yunit = 0.4*sin(th) + nextCoord(2)-0.5;
        
        simdata.agentPlot = arrayfun(@(i) line(xunit, yunit, 'Color', 'green', 'LineWidth',4), 1:simdata.N); 
        simdata.prevCoord = nextCoord;
        hold off;
        simdata.agentisPlotted = true;
    end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    % Callback function to add obstacle image on click
    function toggleObstacle(src, ~) 
        point = ceil(src.CurrentPoint(1, 1:2));  % Get clicked point coordinates
        x_coord = point(1);
        y_coord = point(2);
        fprintf('Button clicked');
        
        subplot(simdata.s1);
        hold on;
        disp(x_coord)
        disp(y_coord)
        if x_coord > 0 && x_coord <= simdata.N && y_coord > 0 && y_coord <= simdata.N
            fprintf('clicked (%d,%d)', x_coord, y_coord);
            % Check if the clicked square has an obstacle
            if simdata.gridData(y_coord, x_coord) == 1
                fprintf('set to 0\n');
                % Remove obstacle image from the clicked square
                simdata.gridData(y_coord, x_coord) = 0;  % Update gridData matrix
                delete(simdata.obstacle_plots_1{y_coord, x_coord});
                delete(simdata.obstacle_plots_2{y_coord, x_coord});
                
            else
                % Add obstacle image to the clicked square
                fprintf('set to 1\n');
                simdata.gridData(y_coord, x_coord) = 1;  % Update gridData matrix
                rh1 = arrayfun(@(i) line([x_coord-1,x_coord], [y_coord-1, y_coord], 'Color', 'black', 'LineWidth',2), 1:simdata.N); 
                rh2 = arrayfun(@(i) line([x_coord-1,x_coord], [y_coord, y_coord-1], 'Color', 'black', 'LineWidth',2), 1:simdata.N); 
                simdata.obstacle_plots_1{y_coord, x_coord} = rh1;
                simdata.obstacle_plots_2{y_coord, x_coord} = rh2;
            end
        end
        hold off;
    end


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
    % Callback functions for the buttons
    function stopCallback(~, ~)
        disp('Stop button pressed');
        % Implement the functionality for "Stop" button here
        simdata.mode = 0;
    end

    function oneStepCallback(~, ~)
        disp('One step button pressed');
        % Implement the functionality for "One step" button here
        simdata.mode = 1;
        simdata.cnt = simdata.cnt + 1;
    end

    function runCallback(~, ~)
        disp('Run button pressed');
        % Implement the functionality for "Run" button here
        simdata.mode = 2;
    end

    function clearCallback(~, ~)
        disp('Clear button pressed');
        % Implement the functionality for "Clear" button here
        simdata.gridData = zeros(simdata.N, simdata.N);
        clf(simdata.f, 'reset');
        simdata.s1 = subplot(1,5,1:2);
        simdata.s2 = subplot(1,5,3:4);
        simdata.s3 = subplot(1, 5, 5);
        plotInputGrid();
    end

    function startCallback(~, ~)
        disp('Start button pressed');
        % Implement the functionality for "Start" button here
        simdata.running = 1;
        uiresume();
    end

    function endCallback(~, ~)
        disp('End button pressed');
        % Implement the functionality for "End" button here
        simdata.running = 0;
    end

    function agentCallback(~, ~)
        pos_free = 0;

        while pos_free == 0

            user_input = inputdlg({'Enter the agents initial position x:',...
                                    'Enter the agents initial position y:',...
                                    'Use random position (0 for no, 1 for yes):',...
                                  }, 'User input', [1 60; 1 60; 1 60], {'0', '0', '0'});
            coords_x = str2double(user_input{1});
            coords_y = str2double(user_input{2});
            is_rand = str2double(user_input{3})
            
            if ~isnumeric(is_rand) || (is_rand~=0 && is_rand~=1) || rem(is_rand, 1) ~= 0
                error('Random position choice must be 0 or 1.');
            end
            
            if is_rand % use position from HMMclass
                pos_free = 1;
            else
                % Check if pos_x and pos_y is a positive integer
                if ~isnumeric(coords_x) || coords_x <= 0 || rem(coords_x, 1) ~= 0
                    error('X must be a positive integer.');
                end
                if ~isnumeric(coords_y) || coords_y <= 0 || rem(coords_y, 1) ~= 0
                    error('Y must be a positive integer.');
                end
                % Check if position is occupied by obstacle
                pos_free = (simdata.gridData(coords_y, coords_x) == 0);
                if pos_free
                    simdata.HMMclass.Robot.Robot_coords = [coords_x, coords_y];
                    disp('Valid initial agent position.');
                end
            end
        end
        
        plotAgent();

    end

end