
nr_meas = 100; %number of measurments
sensors = 5;   %number of sensors: GPS_x, GPS_y, COMPAS_theta, windspeed, wind direction 

meas = zeros(nr_meas, sensors);


% true motion
X = linspace(0,18,nr_meas);
Y = 0.*X;
THETA = atan2(Y,X)
A = 2*ones(1,nr_meas);
PSI = -pi*2/4*ones(1,nr_meas);

%Normally distributed pseudorandom numbers. mean 0, std 1
r_x = .1*randn([1 nr_meas]);
r_y = .1*randn([1 nr_meas]);

%Normally distributed pseudorandom numbers. mean 0, std .1
r_th = .01.*randn([1 nr_meas]);

%Normally distributed pseudorandom numbers. mean 0, std .01
r_a = .001.*randn([1 nr_meas]);
r_psi = .001.*randn([1 nr_meas]);


x = X + r_x;
y = Y + r_y;
theta = THETA + r_th;
a = A + r_a;
psi = PSI + r_psi;

meas = [x' y' theta' a' psi'];
true = [X' Y' THETA' A' PSI'];

clear nr_meas sensors X Y THETA A PSI r_th r_x r_y r_a r_psi x y theta a psi

%[std(x-X) std(theta-THETA)]