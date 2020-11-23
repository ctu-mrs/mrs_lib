% constants

T = 0.01;

% set up desired trajectory

i = 1;

for t = 0:T:0.1
  posd(i) = 0;
  veld(i) = 0;
  accd(i) = 0;
  i = i + 1;
end

for t = 0:T:1.0
  if t < 0.5
    accd(i) = 4;
  else
    accd(i) = -4;
  end
  veld(i) = veld(i-1) + T*accd(i);
  posd(i) = posd(i-1) + T*veld(i) + 0.5*T*T*accd(i);
  i = i + 1;
end

for t = 0:T:0.4
  posd(i) = posd(i-1);
  veld(i) = 0;
  accd(i) = 0;
  i = i + 1;
end

figure();
mpos = posd + 0.001*randn(size(posd));
plot(mpos)                          
title('position with added noise')
xlabel('sample (100Hz)')

% Create idealized system to match filter assumptions
% state is (position, velocity)'
% We are assuming Q, R, and P0 to be diagonal

A = [ 1 T
      0 1 ]

B = [ 0
      T ]

C = [ 1 0 ]

% process variance
Q = [ 1e-6 0
      0 1e-5 ]

% sensor noise variance
R = [ 1e-5 ]

% initial state estimate variance
P0 = [ 1e-4 0
       0 1e-4 ]

% Create some data
state = [ sqrt(P0(1,1))*randn(1)
          sqrt(P0(2,2))*randn(1) ];
for i = 1:length(posd)
 postrue(i) = state(1);
 veltrue(i) = state(2);
 % simulate noisy measurement
 measurement(i) = C*state + sqrt(R(1,1))*randn(1);
 torque(i) = accd(i);
 process_noise = [ sqrt(Q(1,1))*randn(1)
                   sqrt(Q(2,2))*randn(1) ];
 state = A*state + B*accd(i) + process_noise;
end

% Design filter
% Note that we can design filter in advance of seeing the data.
Pm = P0;
for i = 1:1000
 % measurement step
 S = C*Pm*C' + R;
 K = Pm*C'*inv(S);
 Pp = Pm - K*C*Pm;
 % prediction step
 Pm = A*Pp*A' + Q;
end

% Run the filter to create example output
sem = [ 0 0 ]'
for i = 1:length(posd)
 % measurement step
 sep = sem + K*(measurement(i)-C*sem);
 pose(i) = sep(1);
 vele(i) = sep(2);
 % prediction step
 sem = A*sep;
end

% Let's plot the Kalman filter output
figure();
ii = 1:length(pose);
plot(ii,pose,'b',ii,postrue,'r')
plot(ii,measurement,'b',ii,postrue,'r')
plot(ii,vele,'b',ii,veltrue,'r')
legend('KF velocity','true velocity')
xlabel('sample (100Hz)')