
constrained_states = [1; 1; 0; 1; 0; 0] == 1;

n = size(constrained_states, 1);
m = sum(constrained_states);

M = zeros(m, n);
M(:, constrained_states) = eye(m);