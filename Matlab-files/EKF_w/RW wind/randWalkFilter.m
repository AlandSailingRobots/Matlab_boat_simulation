function [theta_out, v_out, psi_out, a_out] = ...
    randWalkFilter(ws_spd, ws_dir, cps_head, gps_spd,Q,R)
% Random walk filtering
%Construct measurments
%convert to correct reference system
windspeed  = ws_spd;
sog = gps_spd;
% winddir = wrapToPi(degtorad(ws_dir));
% yaw = wrapToPi(degtorad(cps_head));

winddir = ws_dir;
yaw = cps_head;

%meas = [ws_spd ws_dir gps_head gps_spd] ;
meas = [windspeed winddir yaw sog]' ;


[xhat, P] = myEKF(1,meas,Q,R);

% xhat = [A PSI theta v]
theta_out = xhat(3);
v_out = xhat(4);
psi_out = xhat(2);
a_out = xhat(1);
