function idx = idx_from_coords(coords, grid_size)
    % Coords are in the (d,s) format, where d determines horizontal
    % movement, and s vertical movement, s=1 the southernmost point
    
    % Coordinates
    % (1,2)   (2,2)    (3,2)
    % (1,1)   (2,1)    (3,1)
    
    % Idx
    % 1   2   3
    % 4   5   6
    
    % (1,2) -> (n_lines - 2)*n_columns + 1 = (2-2)*3 + 1 = 1
    % (1,1) -> (n_lines - 1)*n_columns + 1 = (2-1)*3 + 1 = 4
    % (3,1) -> (n_lines - 1)*n_columns + 3 = (2-1)*3 + 3 = 6
    
    idx = (grid_size(1)-coords(2))*grid_size(2) + coords(1); % (x-1)*nc + y
end

