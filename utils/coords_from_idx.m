function coords = coords_from_idx(idx, grid_size)
    % Coords are in the (d,s) format, where d determines horizontal
    % movement, and s vertical movement, s=1 the southernmost point
    
    % Coordinates
    % (1,2)   (2,2)    (3,2)
    % (1,1)   (2,1)    (3,1)
    
    % Idx
    % 1   2   3
    % 4   5   6
    
    % 1 -> d = rem(1-1, n_columns) + 1 = 1
    %      s = n_lines - (1-1)//n_columns = 2
    
    % 5 -> d = rem(5-1, n_columns) + 1 = 2
    %      s = n_lines - (5-1)//n_columns = 1

    coords_y = rem(idx-1, grid_size(2)) + 1;
    coords_x = grid_size(1) - fix((idx-1)/grid_size(2));
    coords = [coords_y, coords_x];
end