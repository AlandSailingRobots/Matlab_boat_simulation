clear all
makeSignals;    %construct meas and true, containg measurments and true values


nr_m = length(meas(:,1)); %number of measurments
estimates = zeros(nr_m,4);

% First time through the code so do some initialization
xhat = [0;0;10;0];
P = zeros(4,4);
Q = diag([0 1 0 1].*.001);
R = diag([1 1].*1^2);


for i = 1:nr_m
    %function [xhatOut, P, Q, R] = ExtKalman_PhilGoddard(meas,dt,P,xhat,Q,R)
    [xhat, P, Q, R] = ExtKalman_PhilGoddard(meas(i,1:2)',1,P,xhat,Q,R);
    estimates(i,:) = xhat';
end


figure(1)
clf
hold on
plot(true(:,1),true(:,2),'b-')
plot(estimates(:,1),estimates(:,3),'r--')
hold off
