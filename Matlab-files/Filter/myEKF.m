function [xhat_out, P_out] = myEKF(T, meas, u_in)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%states
% x = [x y theta v omega a psi]


%Variables to be saved between calls
persistent xhat P Q R

%first time functions is called, initilize matrices
if isempty(xhat)
    states = 7;
    xhat = zeros(states,1); 
    P = zeros(states,states);
    Q = diag([1 1 1 1 1 1 1]);
    R = diag([1 1 1 .01 .01]) .*1000^2;    
end


%Update linearization
F = df_dx(T,xhat,u_in);

h = [1 0 0 0 0 0 0;     %x
     0 1 0 0 0 0 0;     %y
     0 0 1 0 0 0 0;     %theta
     0 0 0 0 0 1 0;     %a_ap
     0 0 0 0 0 0 1];    %psi_ap
 H = h;
 

%% Predict

%Predict next state estimate:  x_hat_k,k-1 = f(x_hat_k-1,k-1  u_k-1)
%using former state estimate, former control signals
xhat = f(T, xhat, u_in);

%Predict covariance estimate:  P_k,k-1 = F_k-1*P_k-1,k-1*F'_k-1 + Q_k-1
P = F*P*F' + Q;

%% Update

%Innovation - difference between measurement and state prediction
% y_tilde_k = z_k - h(x_hat_k,k-1)
% y_tilde_k = meas - y_hat
y = meas - [xhat(1); xhat(2); xhat(3); xhat(6); xhat(7)];

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
if xhat(3)>pi || xhat(3)<-pi
    xhat(3)
    pause
end
xhat_out = xhat;
P_out = P;

end

