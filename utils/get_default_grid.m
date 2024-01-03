function grid_data = get_default_grid(N)

    if N == 10
        grid_data = load('data/grid_data_10.mat').grid_data;
    elseif N == 20
        grid_data = load('data/grid_data_20.mat').grid_data;
    elseif N == 30
        grid_data = load('data/grid_data_30.mat').grid_data;
    else 
        grid_data = zeros(N);
    end

end