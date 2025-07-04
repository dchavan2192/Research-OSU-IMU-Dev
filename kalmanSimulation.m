% 06/11/2025
% Kalman implementation

% Intialize
% Given intial Eulers, get Quats
clear all
clc


g = 9.81;
phi_0 = 0;
theta_0 = 0.0;
psi_0 = 0;
ss = sprintf('Intial Eulers: %10.2f %10.2f %10.2f  ', phi_0, theta_0, psi_0); disp(ss);
h = 0.1;
% Call Eulet to Quat


quat0 = Euler2Quat1(phi_0,theta_0,psi_0);
quat_km1 = quat0;
P_km1 =1;
R = 0.001*eye(4);
Q = 0.01*eye(4);
H = eye(4);
for t = 0: 0.1: 2*pi

% Read p,q,r, gx,gy,gz
% simulate these
   thsim = 30*pi/180*sin(t);
   ax = -g*sin(thsim);
   ay = 0;
   az = g*cos(thsim);

% compute phi and theta
    theta = asin(ax/g);
    phi = atan(ay/az);
    psi = 0;
% predictor
  % read rates from ios
  % for now, lets generate our ouwn data
  w1 = 0;
  % th = 30*pi/180*sin(t);
  w2 = 30*pi/180*cos(t);
  w3 = 0;

         A_k = h/2*[ 2/h   w3     -w2   w1
                  -w3    2/h    w1    w2
                   w2    -w1    2/h   w3
                  -w1    -w2    -w3   2/h
                 ];
       % State equation
        quat_kp = A_k*quat_km1; % state equation
        
        % Covar  propagation: use gradiant
        B_k = getB(w1,w2,w3);
        P_kp = B_k*P_km1*B_k' + Q;
% corrector

    % compute Kalman filtergain
         K = P_kp*H'*inv(H*P_kp*H' + R);
    % compute x_hat
    % compute quat_m
           
           
         quat_m = Euler2Quat1(phi,theta,psi);
         quat_hat = quat_kp + K*(H*quat_m - quat_kp);   %
    % update Covar matrix
         P_km1 = (eye(4)-K*H)*P_kp;   
% extract Euler from quats
Eul= CompEuler(quat_hat);
ss = sprintf('Eulers:%10.1f %10.2f %10.2f %10.2f  ',t, Eul(1), Eul(2)*180/pi, Eul(3)); disp(ss);
end
% loop up
