function pi0 = calculate_pi0(simdata)

    pi0 = zeros(simdata.n_states, 1);
    
    % Get the states where there are no obstacles
    possible_coords = setdiff(1:simdata.n_states, simdata.obstacles_ids);
    
    pi0(possible_coords) = 1;
    
    % Uniform distribution over free squares
    pi0 = pi0/sum(pi0);
    
end