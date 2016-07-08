clearvars;
addpath('Maps');

[FileName,PathName] = uigetfile('*.jpg','Select the image to get gps points');
    
img = imread([PathName,FileName]);

gps_cal_data = sprintf('gps_cal_data-%s.mat',FileName(1:length(FileName)-4));

if exist([PathName,gps_cal_data], 'file') ~= 2
   error('Calibration data does not exist for this image :\n%s',...
       [PathName,FileName]);
end

load([PathName,gps_cal_data])

h = figure(5); 
imshow(img);hold on;
title('Line : Return to continue, 1 to end selection');
X = []; Y = [];
go_on = 1;
while( go_on == 1)
    [x,y] = ginput(1);
    X = [X x];
    Y = [Y y];
    plot(X,Y,'-ored');
    in = input('Return to continue, 1 to end selection >>> ');
    if(in == 1)
        go_on = 0;
    end
end
hold off;
x = X; y = Y; 
close all;

utm_x = (x-x1)*x_m_pix_ratio+utm_x_1;
utm_y = -(y-y1)*y_m_pix_ratio+utm_y_1;

[lat_out,long_out] = utm2ll(utm_x,utm_y,zone);

prompt = {'FileName:'};
dlg_title = 'Save file under name';
num_lines = 1;
defaultAns = sprintf('gps_point-%s',FileName(1:length(FileName)-4)) ;
defaultans = {defaultAns};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);

text = char(answer(1));


saveText  = sprintf('%s%s.mat',PathName,text);
save(saveText,'utm_x','utm_y','lat_out','long_out');

imshow(img);
hold on
plot(x,y,'-ored');
hold off