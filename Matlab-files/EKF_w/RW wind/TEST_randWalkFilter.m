%% preperations
clear all
load('../trueStates.mat');
%winddir = winddir*180/pi;
%yaw = heading*180/pi;
nr_meas = length(T);


theta = X_true(:,3);
v = X_true(:,4);
A = X_true(:,6);
PSI = X_true(:,7);
W_ap = [A.*cos(PSI-theta)-v A.*sin(PSI-theta)];
%apperent wind speed vector in b-frame
psi_ap = atan2(W_ap(:,2),W_ap(:,1));    %apperent wind angle in b-frame
a_ap = hypot(W_ap(:,1),W_ap(:,2)); 



T_gps = T;
THETA = zeros(1,nr_meas); A=THETA; PSI=THETA; V=THETA;

for i = 1:nr_meas;
    ws_spd = a_ap(i);
    ws_dir = psi_ap(i);
    cps_head = X_true(i,3);
    gps_spd = X_true(i,4);
    
    %% before script in vi
    %nothing to be done
    %% script in vi.
     
    %path(path,'C:\Users\Jon\Dropbox\�SR\Test 2015-05-07\Kopialabview\XbeeSerial - 2010\Serial2010 - Controller');

%     ws_dir = 180-ws_dir;
%     cps_head = 90-cps_head;

    % A PSI theta v
    Q = diag([.01 .01 1 1 ]);
    R = diag([1 1 1 1]);
    
    [theta, v, psi, a] = randWalkFilter(ws_spd, ws_dir, cps_head, gps_spd, Q, R);
    
    %% after script
    %nothing to be done
    
    THETA(i) = theta;
    V(i) = v;
    PSI(i) = psi;
    A(i) = a;
    
end
    %% plots

%     figure(6)
%     plot(T_gps,P(:,1),T_gps,P(:,2),...
%         T_gps,P(:,3),T_gps,P(:,4))
%     legend('A_{true}','\Psi_{true}','\theta','v','location','best')
%     title('P')
    
    
    figure(1)
    plot(T_gps,A,T_gps,X_true(:,6))
    title('Windspeed')
    legend('est','true')
    
    figure(10)
    plot(T_gps,PSI,T_gps,X_true(:,7))
    title('Wind direction')
    legend('est','true')
    
    figure(3)
    plot(T_gps,THETA,T_gps,X_true(:,3))
    title('Theta')
    legend('est','true')
    
    figure(4)
    plot(T_gps,V,T_gps,X_true(:,4))
    title('Boat speed')
    legend('est','true')
    
    figure(7)
    plot(X_true(:,1),X_true(:,2))
    %save('test_randWalk.mat','A','PSI','yaw','sog')
