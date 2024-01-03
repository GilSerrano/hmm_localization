function new_probs = redistribute_probs(probs,impossible_dest)
% Return new probability list of moving to each direction from 1
% coordinate, considering it is not possible to perform the transitions in
% 'impossible_dest'

new_probs = probs;
for i = impossible_dest
    new_probs(i) = 0;
end

total_probs = sum(new_probs);
new_probs = new_probs ./ total_probs;

end

