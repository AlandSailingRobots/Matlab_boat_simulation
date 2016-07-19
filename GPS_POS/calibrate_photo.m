%plot
clear all
[FileName,PathName] = uigetfile('*.jpg','Select the image to calibrate');
    
img = imread([PathName,FileName]);



fig = imshow(img);
title('Select the two known points')
[x1,y1] = ginput(1);

prompt = {'Latitude 1:','Longitude 1:'};
dlg_title = 'Input position first point in degrees decimal';
num_lines = 1;
defaultans = {'0.000','0.000'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
lat1 = str2double(answer(1));
lont1 = str2double(answer(2));
 [utm_x_1,utm_y_1]=ll2utm(lat1,lont1);

%figure(fig);
[x2,y2] = ginput(1);
prompt = {'Latitude 2:','Longitude 2:'};
dlg_title = 'Input position second point in degrees decimal';
num_lines = 1;
defaultans = {'0.000','0.000'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
lat2 = str2double(answer(1));
lont2 = str2double(answer(2));

[utm_x_2,utm_y_2,zone]=ll2utm(lat2,lont2);

x_m_pix_ratio = abs((utm_x_2-utm_x_1)/(x2-x1));
y_m_pix_ratio = abs((utm_y_2-utm_y_1)/(y2-y1));

savefile = sprintf('gps_cal_data-%s.mat',FileName(1:length(FileName)-4));

save([PathName,savefile],'utm_x_1','utm_y_1','utm_x_2','utm_y_2','x1','y1',...
    'x2','y2','x_m_pix_ratio','y_m_pix_ratio','lat1','lont1','lat2','lont2','zone');

close all;
