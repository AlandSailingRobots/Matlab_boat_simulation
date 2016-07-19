function [xhat_out, P_out] = myEKF(T,meas,Q,R)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%states
% x = [A PSI theta v]
%meas = [ws_spd ws_dir gps_head gps_spd] ;

%Variables to be saved between calls
persistent xhat P
states = 4;

%first time functions is called, initilize matrices
if isempty(xhat)
    xhat = zeros(states,1);
    xhat(1) = meas(1);
    xhat(2) = wrapToPi(meas(2)+meas(3));
    xhat(3) = meas(3);
    xhat(4) = meas(4);
    P = ones(states,states);
end
%% Predict
%Update model linearization
F = df_dx(xhat);
%Predict next state estimate:  x_hat_k,k-1 = f(x_hat_k-1,k-1  u_k-1)
%using former state estimate, former control signals
xhat = f(T, xhat);
%Predict covariance estimate:  P_k,k-1 = F_k-1*P_k-1,k-1*F'_k-1 + Q_k-1
P = F*P*F' + Q;

%% Update

%Update measurment linearization
H = dh_dx(xhat);

%Innovation - difference between measurement and state prediction
% y_tilde_k = z_k - h(x_hat_k,k-1)
% y_tilde_k = meas - y_hat
y = meas - h(xhat);

% Correction of angels. Making sure that shortest distance in [-pi pi] is
% used as innovation
for k = [2 3]
    if y(k)>pi
        y(k) = y(k)-2*pi;
    elseif y(k)<-pi
        y(k) = y(k)+2*pi;
    end
end

%Inovataion covariance
% S_k = H_k*P_k,k-1*H'_k + R_k
S = H*P*H' + R;

% Kalman gain
% K_k = P_k,k-1*H'_k* (S_k)^-1
K = P*H'/S;

%Updated state estimate
% x_hat_k,k = x_hat_k,k-1 + K_k*y_tilde_k
xhat = xhat + K*y;

%Updated covariance estimate
P = (eye(states)-K*H)*P;

%% Return state estimate and covarians
%Make sure theta is in [-pi pi]

xhat(2) = wrapToPi(xhat(2));
xhat(3) = wrapToPi(xhat(3));
xhat(1) = abs(xhat(1));
xhat(4) = abs(xhat(4));

xhat_out = xhat;
P_out = P;

end

