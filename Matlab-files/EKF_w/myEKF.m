function [xhat_out, P_out] = myEKF(T, meas, u_in,Q,R)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%states
% x = [x y theta v omega a_t psi_t]


%Variables to be saved between calls
persistent xhat P states %Q R 

%first time functions is called, initilize matrices
if isempty(xhat)
    states = 7;
    xhat = zeros(states,1);
    xhat(1) = meas(1);
    xhat(2) = meas(2);
    xhat(3) = meas(3);
    xhat(4) = meas(4);
    xhat(6) = meas(5);
    xhat(7) = wrapToPi(meas(3)+meas(6));
    P = ones(states,states);
end



%% Predict

%Update model linearization
F = df_dx(T,xhat,u_in);

%Predict next state estimate:  x_hat_k,k-1 = f(x_hat_k-1,k-1  u_k-1)
%using former state estimate, former control signals
xhat = f(T, xhat, u_in);

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
for k = [3 6]  %3-theta, 6-psi_ap in measurments
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
%Make sure angles is in [-pi pi] and speed is positive

xhat(4) = abs(xhat(4));
xhat(6) = abs(xhat(6));

xhat(3) = wrapToPi(xhat(3));
xhat(7) = wrapToPi(xhat(7));

P = diag(diag(P));
xhat_out = xhat;
P_out = P;

end

