function impossible_dest = check_near_obstacles(coord,obstacles, grid_size)
%% Receive a robot coordinate and a list of obstacles coords, and return a list with
% the grid index of neighbouring obstacles of that robot coordinate (0=N, 
% 2=S, 3=W, 4=E, 5=NW, 6=NE, 7=SW, 8=SE). If the cell idx is part of the 
% grid wall, the wall is also included as an obstacle.

% coord: tuple of the current robot position
% obstacles: obstacle matrix
% grid_size: tuple with grid dimensions

% obstacles: a matrix where obstacle coords are set as 1

% Robot coords are (d, s), where s=1 is the southernmost point
% and s = number of lines is the northernmost point. d=0 is the
% east, d = number of columns is west

% Obstacles matrix coords are (x,y): x determines north/south, and y
% determines west/east. Therefore, moving vertically relates s -> x (coord
% 2 of the robot is coord 1 of the grid); moving horizontally relates d ->
% y (coord 1 of the robot is coord 2 of the grid).
% Moreover, in matlab x=1 is the northernmost point of the grid instead of
% the south; we need to calculate n_lines - s + 1 to perform this
% conversion.

% Grid coordinates
% (1,1)   (1,2)   (1,3)
% (2,1)   (2,2)   (2,3)

% Robot coordinates
% (1,2)   (2,2)    (3,2)
% (1,1)   (2,1)    (3,1)

% (d=2, s=1) = (x = 2 - s + 1 ,y = d =2) = (x=2, y=2)
% (d=3, s=2) = (x = 2 - s + 1 ,y = d =3) = (x=1, y=3)

impossible_dest = [];
n_states = grid_size(1)*grid_size(2);

% Robot can't move *N* if:
% it's part of the N wall
if coord(2) == grid_size(1) % s = number of lines
    impossible_dest = [impossible_dest, 1];
% there's an N obstacle
elseif obstacles(grid_size(1)-coord(2)+1 -1, coord(1))
    impossible_dest = [impossible_dest, 1];
end
    
% Robot can't move *S* if:
% it's part of the S wall
if coord(2) == 1 % s = 1
    impossible_dest = [impossible_dest, 2];
% there's an S obstacle
elseif obstacles(grid_size(1)-coord(2)+1 +1, coord(1))
    impossible_dest = [impossible_dest, 2];
end

% Robot can't move *W* if:
% it's part of the W wall
if coord(1) == 1 % d = 1
    impossible_dest = [impossible_dest, 3];
% there's a W obstacle
elseif obstacles(grid_size(1)-coord(2)+1, coord(1)-1)
    impossible_dest = [impossible_dest, 3];
end

% Robot can't move *E* if:
% it's part of the E wall
if coord(1) == grid_size(2) % d = number of columns
    impossible_dest = [impossible_dest, 4];
% there's an E obstacle
elseif obstacles(grid_size(1)-coord(2)+1, coord(1)+1)
    impossible_dest = [impossible_dest, 4];
end


% Robot can't move *NW* if:
% it's part of the N wall
if coord(2) == grid_size(1)
    impossible_dest = [impossible_dest, 5];
% it's part of the W wall
elseif coord(1) == 1
    impossible_dest = [impossible_dest, 5];
% there's an NW obstacle
elseif obstacles(grid_size(1)-coord(2)+1 -1, coord(1)-1)
    impossible_dest = [impossible_dest, 5];
end

% Robot can't move *NE* if:
% it's part of the N wall
if coord(2) == grid_size(1)
    impossible_dest = [impossible_dest, 6];
% it's part of the E wall
elseif coord(1) == grid_size(2)
    impossible_dest = [impossible_dest, 6];
% there's an NE obstacle
elseif obstacles(grid_size(1)-coord(2)+1 -1, coord(1)+1)
    impossible_dest = [impossible_dest, 6];
end

% Robot can't move *SW* if:
% it's part of the S wall
if coord(2) == 1
    impossible_dest = [impossible_dest, 7];
% it's part of the W wall
elseif coord(1) == 1
    impossible_dest = [impossible_dest, 7];
% there's an SW obstacle
elseif obstacles(grid_size(1)-coord(2)+1 +1, coord(1)-1)
    impossible_dest = [impossible_dest, 7];
end

% Robot can't move *SE* if:
% it's part of the S wall
if coord(2) == 1
    impossible_dest = [impossible_dest, 8];
% it's part of the E wall
elseif coord(1) == grid_size(2)
    impossible_dest = [impossible_dest, 8];
% there's an SE obstacle
elseif obstacles(grid_size(1)-coord(2)+1 +1, coord(1)+1)
    impossible_dest = [impossible_dest, 8];
end


    
end

