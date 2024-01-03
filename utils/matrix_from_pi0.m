function pi0_matrix = matrix_from_pi0(pi0, N)
    pi0_matrix = ones(N);
    
    for idx = 1:N*N
        coords = coords_from_idx(idx, [N, N]);
        pi0_matrix(coords(1), coords(2)) = pi0(idx);
    end
            
            


end