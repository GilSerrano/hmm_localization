function pi0 = calculate_pi0(grid_data)

    N = size(grid_data, 1);
    pi0 = ones(N*N, 1);
    
    for i = 1:N
        for j = 1:N
            if grid_data(j,i) == 1 % obstacle
                idx = idx_from_coords([j, i], [N, N]);
                pi0(idx) = 0;
            end
        end
    end
    % Uniform distribution over free squares
    pi0 = pi0/sum(pi0);
    
end