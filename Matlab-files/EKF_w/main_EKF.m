clear all

%load true states and then make measurments from them
load('trueStates.mat'); %loads X_true, u, T, dt

nr_meas = length(T);

%X_true contains true wind. we measure apparent wind...
% X_true(:,1:7) - x y theta v omega a_true psi_true
theta = X_true(:,3);
v = X_true(:,4);
A = X_true(:,6);
PSI = X_true(:,7);
W_ap = [A.*cos(PSI-theta)-v A.*sin(PSI-theta)];
%apperent wind speed vector in b-frame
psi_ap = atan2(W_ap(:,2),W_ap(:,1));    %apperent wind angle in b-frame
a_ap = hypot(W_ap(:,1),W_ap(:,2));      %apperent wind speed velocity in b-frame


%Normaly distributed pseudorandom numbers. mean 0, std ..
r_x     = 10*randn([nr_meas 1]);
r_y     = 10*randn([nr_meas 1]);
r_th    = .01.*randn([nr_meas 1]);
r_v     = .1.*randn([nr_meas 1]);
r_a     = .1.*randn([nr_meas 1]);
r_psi   = .1.*randn([nr_meas 1]);

Q = diag([1 1 1 1 1 .001 .001]).*.1;
R = diag([10 10 .1 .1 .1 .1]).^2;


%Construct measurments
meas = [X_true(:,1:4) a_ap psi_ap];% + [r_x r_y r_th r_v r_a r_psi];
% meas(:,2) = wrapToPi(meas(:,3));
% meas(:,3) = wrapToPi(meas(:,6));


estimates = zeros(nr_meas,7);
P = zeros(nr_meas,7);
%u=zeros(size(u));

for i = 1:nr_meas
    [xhat, p] = myEKF(dt, meas(i,:)', u(i,1:2),Q,R);
    estimates(i,:) = xhat';
    P(i,:) = diag(p)';
end

%%
figure(1)
clf
hold on
plot(X_true(:,1),X_true(:,2),'b-')
plot(estimates(:,1),estimates(:,2),'r')
plot(meas(:,1),meas(:,2),'k.')
hold off

%Plot all state estimates as functions of time
figure(2)
clf
hold on
% plot(T,estimates(:,1),T,estimates(:,2),...
%     T,estimates(:,3),T,estimates(:,4),...
%     T,estimates(:,5),T,estimates(:,6),T,estimates(:,7))
% legend('x','y','\theta','v','\omega','a_{true}','\psi_{true}','location','best')

plot(0,0,0,0,T,estimates(:,3),T,estimates(:,4),...
    T,estimates(:,5),T,estimates(:,6),T,estimates(:,7))
legend('','','\theta','v','\omega','a_{true}','\psi_{true}','location','best')


title('Estimated states')
hold off

%Plot of "true" states as functions of time
figure(3)
clf
hold on
plot(0,0,0,0,...
    T,X_true(:,3),T,X_true(:,4),...
    T,X_true(:,5),T,X_true(:,6),T,X_true(:,7))
legend('','','\theta','v','\omega','a_{true}','\psi_{true}','location','best')
% plot(T,X_true(:,1),T,X_true(:,2),...
%     T,X_true(:,3),T,X_true(:,4),...
%     T,X_true(:,5),T,X_true(:,6),T,X_true(:,7))
% legend('x','y','\theta','v','\omega','a_{true}','\psi_{true}','location','best')
title('"True" states')
hold off


%Plot all states errors as functions of time
figure(4)
clf
hold on
% plot(T,X_true(:,1)-estimates(:,1),T,X_true(:,2)-estimates(:,2),...
%     T,X_true(:,3)-estimates(:,3),T,X_true(:,4)-estimates(:,4),...
%     T,X_true(:,5)-estimates(:,5),T,X_true(:,6)-estimates(:,6),...
%     T,X_true(:,7)-estimates(:,7))
plot(T,X_true(:,3)-estimates(:,3),T,X_true(:,4)-estimates(:,4),...
    T,X_true(:,5)-estimates(:,5),T,X_true(:,6)-estimates(:,6),...
    T,X_true(:,7)-estimates(:,7))
legend('\theta','v','\omega','a_{true}','\psi_{true}','location','best')
title('Error')
hold off



%Plot P
figure(5)
clf
plot(T,P(:,1),T,P(:,2),T,P(:,3),T,P(:,4),T,P(:,5),T,P(:,6),T,P(:,7))
title('P')
legend('x','y','\theta','v','\omega','a_{true}','\psi_{true}','location','best')


%% Plot one variable
% figure(6)
% str = {'x','y','\theta','v','\omega','a_{true}','\psi_{true}' };
% 
% var2plot = 7;
% 
% if var2plot <= 4
%     h = plot(T,X_true(:,var2plot),'-b',...
%      T,meas(:,var2plot),'.k',...
%      T,estimates(:,var2plot),'-r');
% legend(str(var2plot),'measurement')
% legend('location','best')
% else
%     plot(T,X_true(:,var2plot),'-b',...
%      T,estimates(:,var2plot),'-r')
% legend(str(var2plot),'estimate')
% legend('location','best')
% end

figure(6)
plot(T,X_true(:,7),'-b', T,estimates(:,7),'-r',T,wrapToPi(theta+psi_ap))
legend('true','estimate','theta+psi_{ap}')
legend('location','best')
title('Wind direction')



figure(1)


