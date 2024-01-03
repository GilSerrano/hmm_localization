function B = build_matrix_B(p_error, n_sensors, obstacles, grid_size)
% Build emission matrix B from a chosen number of sensors and their error
% probability.

n_states = prod(size(obstacles));

if n_sensors == 4
    % One-hot encode possible sensor readings
    N = [1, 0, 0, 0]; %1
    S = [0, 1, 0, 0]; %2
    W = [0, 0, 1, 0]; %3
    E = [0, 0, 0, 1]; %4
    NS = [1, 1, 0, 0]; %5
    NW = [1, 0, 1, 0]; %6
    NE = [1, 0, 0, 1]; %7
    SW = [0, 1, 1, 0]; %8
    SE = [0, 1, 0, 1]; %9
    WE = [0, 0, 1, 1]; %10
    NSW = [1, 1, 1, 0]; %11
    NSE = [1, 1, 0, 1]; %12
    NWE = [1, 0, 1, 1]; %13
    SWE = [0, 1, 1, 1]; %14
    NSWE = [1,1,1,1]; %15
    X = [0,0,0,0]; %16
    
    % Initialize B matrix (n_states x n sensor combinations)
    B = zeros(n_states, 16);
    
    % Go through each state coordinate
    for i = 1: n_states
        % Convert idx into coordinate
        coords = coords_from_idx(i, grid_size);
        
        % Find the surrounding obstacles
        impossible_dest = check_near_obstacles(coords,obstacles, grid_size);
        
        % Ideal sensor readings (assuming no errors)
        sensor_readout = zeros(1, 8);
        for j = impossible_dest
            sensor_readout(j) = 1;
        end
        sensor_readout = sensor_readout(1:4); % crop to N, S, E and W only
        
        % Fill in B matrix columns
        B(i, 1) = get_probability(N, sensor_readout, p_error);
        B(i, 2) = get_probability(S, sensor_readout, p_error);
        B(i, 3) = get_probability(W, sensor_readout, p_error);
        B(i, 4) = get_probability(E, sensor_readout, p_error);
        B(i, 5) = get_probability(NS, sensor_readout, p_error);
        B(i, 6) = get_probability(NW, sensor_readout, p_error);
        B(i, 7) = get_probability(NE, sensor_readout, p_error);
        B(i, 8) = get_probability(SW, sensor_readout, p_error);
        B(i, 9) = get_probability(SE, sensor_readout, p_error);
        B(i, 10) = get_probability(WE, sensor_readout, p_error);
        B(i, 11) = get_probability(NSW, sensor_readout, p_error);
        B(i, 12) = get_probability(NSE, sensor_readout, p_error);
        B(i, 13) = get_probability(NWE, sensor_readout, p_error);
        B(i, 14) = get_probability(SWE, sensor_readout, p_error);
        B(i, 15) = get_probability(NSWE, sensor_readout, p_error);
        B(i, 16) = get_probability(X, sensor_readout, p_error);
    end
    
elseif n_sensors == 8
    % One-hot encode possible sensor readings    
    SignalMatrix = dec2bin(0:2^8-1)-'0'; % contains all combinations of 8 sensor readouts
    
    % Initialize B matrix (n_states x n sensor combinations)
    B = zeros(n_states, 2^8);
    
    % Go through each state coordinate
    for i = 1: n_states
        % Convert idx into coordinate
        coords = coords_from_idx(i, grid_size);
        
        % Find the surrounding obstacles
        impossible_dest = check_near_obstacles(coords,obstacles, grid_size);
        
        % Ideal sensor readings (assuming no error)
        sensor_readout = zeros(1, 8);
        for j = impossible_dest
            sensor_readout(j) = 1;
        end
        
        % Fill in B matrix columns
        for Bcolumn = 1:size(SignalMatrix, 1)
            B(i, Bcolumn) = get_probability(SignalMatrix(Bcolumn, :), sensor_readout, p_error);
        end
    end
    
else
    error ("Only 4 or 8 robot sensors are supported.");
end


end

function prob = get_probability(dir_encode, sensor_encode, err_prob)
    % Calculate probability of a specific sensor reading combination
    % dir_encode: ideal sensor combination 
    % sensor_encode: measured sensor reading
    % err_prob: list of error probability of each sensor
    
    entry = abs(dir_encode - sensor_encode); % Desired - true
    correct_measure = 1- err_prob; % Probability of no error
    prob = entry;
    
    for i=1:length(entry) % Go through each sensor in the list
        if entry(i) == 0 % if desired = true of this sensor
            prob(i) = correct_measure(i); % set as probability of no error
        else
            prob(i) = err_prob(i); % set as probability of error
        end
    end
    prob = prod(prob); % multiply the probabilities of all sensors in the lists
end