addpath('./error_ellipse')
clear;
close all;

figure;
hold on;
grid on;
axis square;
xlim([-20, 20]);
ylim([-20, 20]);
zlim([-20, 20]);
title("measurements");

dt = 1;
n = 6;
if n == 6
  A = [eye(3), dt*eye(3);
       zeros(3),  eye(3)];
  ground_truth = [13; -3; -3; -1; 1; 1];
  constrained_states = [1; 1; 1; 0; 0; 0];
else
  A = eye(3);
  ground_truth = [-3; -3; -3];
  constrained_states = [1; 1; 1];
end
x = zeros(n, 1);
P = 666*eye(n);
Q = 1.0*eye(n);
% Q = 0.1*eye(n);

for it = 1:20
  ground_truth = A*ground_truth;
  gt_pos = ground_truth(1:3);
  
  % [bases, origin, m] = plane_meas(gt_pos);
  % [bases, origin, m] = line_meas(gt_pos);
  [bases, origin, m, R_i] = plane_velocity_meas(ground_truth);
  if size(bases,1) == 3
    M = constraint_transform(constrained_states);
  else
    M = eye(6);
  end
  
  nbases = null(bases');
  % op = nbases'*origin;
  
  H = nbases'*M;

  % dev = tan(deg2rad(0.5))*15; % error of 1 degree at max. range of 15m
  dev = 0.1;
  % z = op + dev*randn(size(op,1),1)
  % z = nbases'*origin + dev*randn(size(size(nbases,2),1),1)
  z = nbases'*origin;
  R = nbases'*R_i*nbases
  % R = dev*eye(size(z,1));
  % R_t = bases*R*bases'
  % R_tn = nbases'*R_t*nbases
  
  [x, P] = kf_predict(A, x, P, Q);
  [x, P] = kf_correct(H, x, P, z, R);
  
  op3d = nbases*z;

  if (exist('OCTAVE_VERSION', 'builtin') == 0)
    if (exist('hb') ~= 0)
      hb.delete
    end
    if (exist('hn') ~= 0)
      hn.delete
    end
    if (exist('ho') ~= 0)
      ho.delete
    end
    if (exist('he') ~= 0)
      he.delete
    end
  end


  hb = plot_meas_z(bases, z);
  if m<3
    hn = plot_meas(nbases, [0; 0; 0]);
    ho = plot3(op3d(1), op3d(2), op3d(3), 'o');
  else
    hn = plot_meas(bases(4:6,2:3), origin(4:6,1));
    ho = plot3(origin(1), origin(2), origin(3), 'o');
  end



  plot3(ground_truth(1), ground_truth(2), ground_truth(3), '*');
  he = error_ellipse(P(1:3,1:3),x(1:3));

  fprintf("it %d: %f\n", it, norm(x-ground_truth));
  egn = eig(P)';
  % dt = det(P)'
  % inside = egn(1) - norm(e)^2;
  e = x - ground_truth;
  e2 = e.^2;
  e_i = P\e2;
  n_e_i = sqrt(norm(e_i))
  outside = n_e_i > 1;
  if (outside)
    e_i
    % norm(e)^2
    % egn(1)
  end

  pause(1)
end

x
egn
ground_truth

plot3(0, 0, 0, 'x', "MarkerSize", 15);
