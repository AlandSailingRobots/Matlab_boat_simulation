clear all

%load true states and then make measurments from them
load('trueStates.mat'); %loads X_true, u, T, dt

nr_meas = length(T);z

%X_true contains true wind. we measure apparent wind...
% X_true(:,1:7) - x y theta v omega a_true psi_true
theta = X_true(:,3);
X_true(:,4)= X_true(:,4)*.5;
v = X_true(:,4);
A = X_true(:,6);
PSI = X_true(:,7);

W_ap = [A.*cos(PSI-theta)-v A.*sin(PSI-theta)];
%apperent wind speed vector in b-frame
psi_ap = atan2(W_ap(:,2),W_ap(:,1));    %apperent wind angle in b-frame
a_ap = hypot(W_ap(:,1),W_ap(:,2));      %apperent wind speed velocity in b-frame

%psi_ap = unwrap(psi_ap);

%Normaly distributed pseudorandom numbers. mean 0, std ..
% r_x     = 10*randn([nr_meas 1]);
% r_y     = 10*randn([nr_meas 1]);
r_th    = .1.*randn([nr_meas 1]);
r_v     = .1.*randn([nr_meas 1]);
r_a     = .1.*randn([nr_meas 1]);
r_psi   = .5.*randn([nr_meas 1]);

Q = diag([.01 .01 .01 .01 ]).*1;
R = diag([1 1 1 1]).^2;






%Construct measurments
meas = [a_ap psi_ap X_true(:,3:4) ] + [r_a r_psi r_th r_v];
meas(:,2) = wrapToPi(meas(:,2));
meas(:,3) = wrapToPi(meas(:,3));

estimates = zeros(nr_meas,4);
P = zeros(nr_meas,4);


for i = 1:nr_meas
    
    
        
    if i >160
       a= 1 + 1; 
    end
    
    [xhat, p] = myEKF(dt, meas(i,:)', Q,R);
    estimates(i,:) = xhat';
    P(i,:) = diag(p)';

    
end


% figure(1)
% clf
% hold on
% plot(X_true(:,1),X_true(:,2),'b-')
% plot(estimates(:,1),estimates(:,2),'r')
% plot(meas(:,1),meas(:,2),'k.')
% hold off
% 
%Plot all state estimates as functions of time
figure(1)
clf
hold on
plot(T,estimates(:,1),T,estimates(:,2),...
    T,estimates(:,3),T,estimates(:,4))
legend('A_{true}','\Psi_{true}','\theta','v','location','best')

title('Estimated states')
hold off

%Plot of "true" states as functions of time
figure(2)
clf
hold on
plot(T,X_true(:,6),T,X_true(:,7),...
    T,X_true(:,3),T,X_true(:,4))
legend('a_{true}','\psi_{true}','\theta','v','location','best')
% plot(T,X_true(:,1),T,X_true(:,2),...
%     T,X_true(:,3),T,X_true(:,4),...
%     T,X_true(:,5),T,X_true(:,6),T,X_true(:,7))
% legend('x','y','\theta','v','\omega','a_{true}','\psi_{true}','location','best')
title('"True" states')
hold off
% 
% 
% %Plot all states errors as functions of time
% figure(4)
% clf
% hold on
% plot(T,X_true(:,1)-estimates(:,1),T,X_true(:,2)-estimates(:,2),...
%     T,X_true(:,3)-estimates(:,3),T,X_true(:,4)-estimates(:,4),...
%     T,X_true(:,5)-estimates(:,5),T,X_true(:,6)-estimates(:,6),...
%     T,X_true(:,7)-estimates(:,7))
% legend('x','y','\theta','v','\omega','a_{true}','\psi_{true}','location','best')
% title('Error')
% hold off
% 
% 

%Plot P
figure(5)
clf
plot(T,P(:,1),T,P(:,2),T,P(:,3),T,P(:,4))
title('P')
legend('a_{true}','\psi_{true}','\theta','v','location','best')


%% Plot one variable
figure(3)
str = 'a_{true}';

var2plot = 6;



    plot(T,X_true(:,var2plot),'-b',T,estimates(:,1),'-r')
legend(str,'estimate')
legend('location','best')



figure(4)
str = '\psi_{true}';

var2plot = 7;
for i = 1:length(T)
hh(i,:) = h(estimates(i,:));
end
    plot(T,X_true(:,var2plot),'-b',T,estimates(:,2),'-r',T,meas(:,2),T,hh(:,2),'k')
legend(str,'estimate','appW. meas.','appW. meas. prediction')
legend('location','best')

% 
% 
% 
% figure(1)


