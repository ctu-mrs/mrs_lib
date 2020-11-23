clear;
close all;

figure;
hold on;
grid on;
axis square;
xlim([-3, 3]);
ylim([-3, 3]);
zlim([-3, 3]);
title("measurements");

dt = 1;
n = 6;
if n == 6
  A = [eye(3), dt*eye(3);
       zeros(3),  eye(3)];
  ground_truth = [-3; -3; -3; 1; 1; 1];
  constrained_states = [1; 1; 1; 0; 0; 0];
else
  A = eye(3);
  ground_truth = [-3; -3; -3];
  constrained_states = [1; 1; 1];
end
x = zeros(n, 1);
P = 666*eye(n);
Q = eye(n);

for it = 1:20
  ground_truth = A*ground_truth;
  gt_pos = ground_truth(1:3);
  
  [bases, origin, m] = plane_meas(gt_pos);
  M = constraint_transform(constrained_states);
  
  nbases = null(bases');
  op = nbases'*origin;
  
  H = nbases'*M;
  z = op;
  R = eye(m);
  
  [x, P] = kf_predict(A, x, P, Q);
  [x, P] = kf_correct(H, x, P, z, R);
  
  op3d = nbases*op;
  plot_meas(bases, origin);
  plot_meas(nbases, [0; 0; 0]);
  plot3(op3d(1), op3d(2), op3d(3), 'o');
  fprintf("it %d: %f\n", it, norm(x-ground_truth));
endfor

x
P
ground_truth

plot3(0, 0, 0, 'x', "MarkerSize", 15);
