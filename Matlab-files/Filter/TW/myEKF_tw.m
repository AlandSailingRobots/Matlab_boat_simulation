function [xhat_out, P_out] = myEKF_tw(T, meas, u_in)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%states
% x = [x y theta v omega a psi]


%Variables to be saved between calls
persistent xhat P Q R

%first time functions is called, initilize matrices
if isempty(xhat)
    states = 7;
    xhat = ones(states,1); 
    P = zeros(states,states);
    Q = diag([1 1 1 1 1 1 1]);
    R = diag([1 1 1 1 1]) .*1^2;    
end


%Update linearization
F = df_dx_tw(T,xhat,u_in);
H = dh_dx_tw(xhat);
 

%% Predict

%Predict next state estimate:  x_hat_k,k-1 = f(x_hat_k-1,k-1  u_k-1)
%using former state estimate, former control signals
xhat = f_tw(T, xhat, u_in);

%Predict covariance estimate:  P_k,k-1 = F_k-1*P_k-1,k-1*F'_k-1 + Q_k-1
P = F*P*F' + Q;

%% Update

%Innovation - difference between measurement and state prediction
% y_tilde_k = z_k - h(x_hat_k,k-1)
% y_tilde_k = meas - y_hat
y = meas - h_tw(xhat);

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
P = (eye(7)-K*H)*P;

%% Return state estimate and covarians
%Make sure theta is in [-pi pi]

xhat(3) = wrapToPi(xhat(3));
xhat(7) = wrapToPi(xhat(7));
% if xhat(3)>pi || xhat(3)<-pi
%     xhat(3)
%     pause
% end
xhat_out = xhat;
P_out = P;

end

