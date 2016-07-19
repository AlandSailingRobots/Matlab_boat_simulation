
gps_point = [60.102500,  19.920000
 60.101675,  19.920471
 60.100705,  19.920993
 60.099852,  19.921348
 60.100705,  19.920993
 60.101675,  19.920471
 60.102500,  19.920000];

lat_out = gps_point(:,1);
long_out = gps_point(:,2);

[utm_x,utm_y]=ll2utm(lat_out,long_out);

save('gps_point-test2206.mat','utm_x','utm_y','lat_out','long_out');
