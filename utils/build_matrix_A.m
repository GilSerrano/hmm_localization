function A = build_matrix_A(direction_probs, grid_size, obstacles, n_moves)
% Build transition matrix A from chosen probabilities of going in each direction
% direction_probs: dictionary with the probability of moving in each direction
% index      direction
% 1          N
% 2          S
% 3          W
% 4          E
% 5          NW diagonal
% 6          NE diagonal
% 7          SW diagonal
% 8          SE diagonal
% 9          X not moving

% Matrix A respects the user defined probabilities except if: the position
% belongs next to a wall; the neighbouring positions are occupied by
% obstacles. In these cases, the impossible transitions are set to p=0, and
% the remaining are calculated using proportional values to the user
% defined ones.

% A is a n_states x n_states matrix; states can be accessed by their idx in
% the environment

% (1->1) (1->2) ... (1-> nstates)
% (2->1) (2->2) ... (2-> nstates)
% ...
% (nstates->1) (nstates->2) ... (nstates->nstates)

% Each idx is a grid coordinate. In display coordinates, assumin nstates=10:
% ([1,10]->[1,10]) ([1,10]->[2,10]) ... ([1,10]-> [10,1])
% ([2,10]->[1,10]) ([2,10]->[2,10]) ... ([2,10]-> [10,1])
% ...
% ([10,1]->[1,10]) ([10,1]->[2,10]) ... ([10,1]->[10,1])

% The larger the idx, the more south/east we are.

n_states = grid_size(1)*grid_size(2);
A = sparse(n_states, n_states);

% Go through each coordinate
for i = [1:n_states]
    
    % Convert idx into coordinates
    coords = coords_from_idx(i, grid_size);
    
    % Find out impossible directions to move to
    impossible_dest = check_near_obstacles(coords,obstacles, grid_size);
    
    if n_moves == 4
        impossible_dest(impossible_dest>4) = [];
    end
    
    % If there are impossible directions, redistribute probabilities
    if isempty(impossible_dest)
        probs = direction_probs;
    else
        probs = redistribute_probs(direction_probs,impossible_dest);
    end
    
    % Populate matrix A
    
    if n_moves == 8
        A(i,i) = probs(9); % diagonal corresponds to staying in the same coord
        % NW corner
        if coords == [1, grid_size(1)]
            A(i, i + grid_size(2)) = probs(2); %S
            A(i, i+1) = probs(4); % E
            A(i, i+grid_size(2)+1) = probs(8); % SE
        % NE corner
        elseif coords == [grid_size(2),grid_size(1)]
            A(i, i + grid_size(2)) = probs(2); %S
            A(i, i-1) = probs(3); % W
            A(i, i+grid_size(2)-1) = probs(7); % SW
        % SW corner
        elseif coords == [1, 1]
            A(i, i - grid_size(2)) = probs(1); %N
            A(i, i+1) = probs(4); % E
            A(i, i-grid_size(2)+1) = probs(6); % NE
        % SE corner
        elseif coords == [grid_size(2), 1]
            A(i, i - grid_size(2)) = probs(1); %N
            A(i, i-1) = probs(3); % W
            A(i, i-grid_size(2)-1) = probs(5); % NW
        % N wall
        elseif coords(2) == grid_size(1)
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i-1) = probs(3); % W
            A(i, i+1) = probs(4); % E
            A(i, i+grid_size(2)-1) = probs(7); % SW
            A(i, i+grid_size(2)+1) = probs(8); % SE
        % S wall
        elseif coords(2) == 1
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i-1) = probs(3); % W
            A(i, i+1) = probs(4); % E
            A(i, i-grid_size(2)-1) = probs(5); % NW
            A(i, i-grid_size(2)+1) = probs(6); % NE
        % W wall
        elseif coords(1) == 1
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i+1) = probs(4); % E
            A(i, i-grid_size(2)+1) = probs(6); % NE 
            A(i, i+grid_size(2)+1) = probs(8); % SE 
        % E wall
        elseif coords(1) == grid_size(2)
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i-1) = probs(3); % W
            A(i, i-grid_size(2)-1) = probs(5); % NW 
            A(i, i+grid_size(2)-1) = probs(7); % SW
        else
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i-1) = probs(3); % W
            A(i, i+1) = probs(4); % E
            A(i, i-grid_size(2)-1) = probs(5); % NW
            A(i, i-grid_size(2)+1) = probs(6); % NE
            A(i, i+grid_size(2)-1) = probs(7); % SW
            A(i, i+grid_size(2)+1) = probs(8); % SE
        end
        
        
    elseif n_moves == 4
       A(i,i) = probs(5); % diagonal corresponds to staying in the same coord
        % NW corner
        if coords == [1, grid_size(1)]
            A(i, i + grid_size(2)) = probs(2); %S
            A(i, i+1) = probs(4); % E
        % NE corner
        elseif coords == [grid_size(2),grid_size(1)]
            A(i, i + grid_size(2)) = probs(2); %S
            A(i, i-1) = probs(3); % W
        % SW corner
        elseif coords == [1, 1]
            A(i, i - grid_size(2)) = probs(1); %N
            A(i, i+1) = probs(4); % E
        % SE corner
        elseif coords == [grid_size(2), 1]
            A(i, i - grid_size(2)) = probs(1); %N
            A(i, i-1) = probs(3); % W
        % N wall
        elseif coords(2) == grid_size(1)
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i-1) = probs(3); % W
            A(i, i+1) = probs(4); % E
        % S wall
        elseif coords(2) == 1
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i-1) = probs(3); % W
            A(i, i+1) = probs(4); % E
        % W wall
        elseif coords(1) == 1
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i+1) = probs(4); % E
        % E wall
        elseif coords(1) == grid_size(2)
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i-1) = probs(3); % W
        else
            A(i, i-grid_size(2)) = probs(1); % N
            A(i, i+grid_size(2)) = probs(2); % S
            A(i, i-1) = probs(3); % W
            A(i, i+1) = probs(4); % E
        end
        
    end
end
end

