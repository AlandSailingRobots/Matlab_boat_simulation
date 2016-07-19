
clear all
%import logdata using graphical import tool and save to .mat file

load('data_1280end.mat');
gps_time = datevec(gps_time);

% rs_pos and ss_pos - only zeros
% wpt_cur - onlye ones


%timestamp vector in seconds. First timestamp - 0 seconds
T = gps_time(:,4)*3600 + gps_time(:,5)*60 +gps_time(:,6) ...
    - gps_time(:,4)*3600 - gps_time(1,5)*60 - gps_time(1,6);


figure(1)
plot(gps_lon,gps_lat,gps_lon(500),gps_lat(500),'*')

figure(2)
plot(T, gps_head,...
    T, gps_spd)
legend('gps_{head}','gps_{spd}')

figure(3)
plot(T, rc_cmd, T, sc_cmd)
legend('rc_{cmd}','sc_{cmd}')


figure(4)
plot(T, ws_dir, T, ws_spd, T, ws_tmp)
legend('ws_{dir}','ws_{spd}','ws_{tmp}')