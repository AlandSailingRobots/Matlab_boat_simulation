function [x, y] = ll2m( lat1, lon1, lat2, lon2 )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

R = 6371000; %earth radius in meters


lat1 = lat1/180*pi;
lon1 = lon1/180*pi;

lat2 = lat2/180*pi;
lon2 = lon2/180*pi;

    function [d, sla, slo]= fun(la1,lo1,la2,lo2)
        dla = la2-la1;
        sla = sign(dla);
        dlo = lo2-lo1;
        slo = sign(dlo);
        a = sin(dla/2)^2 + cos(la1) * cos(la2) * sin(dlo/2)^2;
        c = 2*atan2(sqrt(a),sqrt(1-a));
        d = R*c;
    end

[x, sla, slo] = fun(lat1,lon1,lat1,lon2);
x = slo*x;
[y, sla, slo] = fun(lat1,lon1,lat2,lon1);
y = sla*y;

end

