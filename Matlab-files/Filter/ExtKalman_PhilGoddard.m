function [xhatOut, P, Q, R] = ExtKalman_PhilGoddard(meas,dt,P,xhat,Q,R)
% This Embedded MATLAB Function implements an extended Kalman filter used
% for object tracking.
%
% The states of the process are given by
% x = [x_position; x_velocity; y_position; y_velocity];
%
% and the measurements are given by
% y = [range; bearing]
%
% where
% range = sqrt(x_position^2 + y_position^2)
% bearing = atan2(y_position/x_position)

% Author: Phil Goddard (phil@goddardconsulting.ca)
% Date: Q2, 2011.

% Define storage for the variables that need to persist
% between time periods.


% persistent P xhat Q R
% if isempty(P)
%    % First time through the code so do some initialization
%    xhat = [0;0;0;0];
%    P = zeros(4,4);
%    Q = diag([0 .1 0 .1]);
%    R = diag([50^2 0.005^2]);
% end


% Calculate the Jacobians for the state and measurement equations
F = [1 dt 0 0;0 1 0 0;0 0 1 dt;0 0 0 1];
%rangeHat = sqrt(xhat(1)^2+xhat(3)^2);
%bearingHat = atan2(xhat(3),xhat(1));
yhat = [xhat(1); xhat(3)];
H = [1 0 0 0;
     0 0 1 0];
% Propogate the state and covariance matrices
xhat = F*xhat;
P = F*P*F' + Q;
% Calculate the Kalman gain
K = P*H'/(H*P*H' + R);
% Calculate the measurement residual
resid = meas - yhat;
% Update the state and covariance estimates
xhat = xhat + K*resid;
P = (eye(size(K,1))-K*H)*P;
% Post the results
xhatOut = xhat;