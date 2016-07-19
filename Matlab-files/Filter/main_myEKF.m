clear all
makeSignals;


nr_m = length(meas(:,1)); %number of measurments
estimates = zeros(nr_m,7);


for i = 1:nr_m
    %[xhat_out] = myEKF(T, meas, u_in)
    [xhat, P] = myEKF(.1, meas(i,1:5)', [0 pi/4]);
    estimates(i,:) = xhat';
end


figure(1)
clf
hold on
plot(true(:,1),true(:,2),'b-')
plot(estimates(:,1),estimates(:,2),'r')
plot(meas(:,1),meas(:,2),'k*')
hold off
