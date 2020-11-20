pkg load control

ground_truth = [1; 1; 1];

figure;
hold on;
grid on;
xlim([-3, 3]);
ylim([-3, 3]);
zlim([-3, 3]);
title("measurements");

A = eye(3);
x = [0;0;0];
P = 666*eye(3);

for it = 1:10
  [bases, origin] = plane_meas(ground_truth);
  m = 3-size(bases, 2);
  
  nbases = null(bases');
  op = nbases'*origin;
  
  H = nbases';
  z = op;
  R = eye(m);
  
  [x, P] = kf_correct(A, H, x, P, z, R)
  
  op3d = nbases*op;
  plot_meas(bases, origin);
  plot_meas(nbases, [0; 0; 0]);
  plot3(op3d(1), op3d(2), op3d(3), 'o');
endfor

plot3(0, 0, 0, 'x', "MarkerSize", 15);
