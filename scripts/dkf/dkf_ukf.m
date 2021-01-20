% A script for comparing the DKF and UKF for a simple scenario with line measurements of a 3D point
% (optionally a moving 3D point). Author: vrbamato@fel.cvut.cz

clear;
addpath('ukf');

% Signal to noise ratio (probably not according to the definition, just a parameter)
SNR = 100;

dt = 1; % shouldn't really matter much
n = 3;  % number of states - 3 for position only, 6 for position and velocity
% Generate the relevant matrices and initial conditions according to the number of states
if n == 6
  A = [eye(3), dt*eye(3);
       zeros(3),  eye(3)];
  constrained_states = [1; 1; 1; 0; 0; 0];
else
  A = eye(3);
  constrained_states = [1; 1; 1];
end
ground_truth = [SNR*rand(3, 1); rand(n-3, 1)];
x = ground_truth; % assume the initial state is known (otherwise the UKF tends to diverge more often)
P = SNR*eye(n);
Q = eye(n);

xd = x;
Pd = P;
xu = x;
Pu = P;

Nits = 1e3; % number of iterations

% Define arrays for storing the results
gts = zeros(n, Nits); % ground truths

xds = zeros(n, Nits); % DKF state estimates
xus = zeros(n, Nits); % UKF state estimates

dds = zeros(1, Nits); % DKF covariance determinants
dus = zeros(1, Nits); % UKF covariance determinants

%% THE LOOP
for it = 1:Nits
  % Update the ground truth state vector
  if it ~= 1
    ground_truth = A*ground_truth + normrnd(zeros(n, 1), ones(n, 1));
  end
  gt_pos = ground_truth(1:3);
  origin = SNR*rand(3, 1); % origin of the measurement (offset of the line, position of the camera etc.)
  
  %% GENERATE A NEW MEASUREMENT
%  bases = gt_pos - origin + normrnd(3, 1); % this noise better corresponds to a practical application, but not to the DKF and UKF measurement noise models
  bases = gt_pos - origin;
  bases = bases/norm(bases);
  origin = origin + normrnd(zeros(3,1), ones(3,1)); % this one better corresponds to the DKF and UKF measurement noise models
  
  %% DKF
  m = 2; % dimension of the measurement
  M = constraint_transform(constrained_states);
  
  nbases = null(bases');
  op = nbases'*origin;
  
  H = nbases'*M;
  zd = op;
  Rd = eye(m);
  
  [xd, Pd] = kf_predict(A, xd, Pd, Q);
  [xd, Pd] = kf_correct(H, xd, Pd, zd, Rd);
  
  %% UKF
  zu = bases;
  fstate = @(x) A*x;
  hmeas = @(x) (x(1:3) - origin)/norm(x(1:3) - origin);
  Ru = eye(3);
  [xu, Pu] = ukf(fstate, xu, Pu, hmeas, zu, Q, Ru);
  
  %% save the results
  gts(:, it) = ground_truth;
  xds(:, it) = xd;
  xus(:, it) = xu;
  dds(it) = det(Pd);
  dus(it) = det(Pu);
end

% calculate the RMSE errors
eds = sqrt(sum((xds - gts).^2));
eus = sqrt(sum((xus - gts).^2));

% plot the results
subplot(2, 1, 1);
pleds = plot(1:Nits, eds, 1:Nits, eus);
legend("DKF RMSE", "UKF RMSE");
xlabel("iteration");
ylabel("error");

subplot(2, 1, 2);
plot(1:Nits, dds, 1:Nits, dus);
legend("DKF covariance", "UKF covariance");
xlabel("iteration");
ylabel("covariance determinant");
