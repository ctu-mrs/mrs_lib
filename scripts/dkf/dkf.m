pkg load control

gt_pos = [-3; -3; -3];
gt_vel = [1; 1; 1];

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
A = [eye(3), dt*eye(3);
     zeros(3),  eye(3)];
x = [zeros(3, 1), eye(3, 1)];
P = 666*eye(n);

for it = 1:5
  [bases, origin, M] = point_meas(gt_pos, n);
  m = size(gt_pos, 1)-size(bases, 2);
  
  nbases = eye(3);
  if size(bases, 2) > 0
    nbases = null(bases');
  endif
  op = nbases'*origin;
  
  H = nbases'*M;
  z = op;
  R = eye(m);
  
  [x, P] = kf_correct(A, H, x, P, z, R)
  
  op3d = nbases*op;
  plot_meas(bases, origin);
  plot_meas(nbases, [0; 0; 0]);
  plot3(op3d(1), op3d(2), op3d(3), 'o');
  ground_truth = A*[gt_pos; gt_vel];
  gt_pos = ground_truth(1:3);
endfor

plot3(0, 0, 0, 'x', "MarkerSize", 15);
