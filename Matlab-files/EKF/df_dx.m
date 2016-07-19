function [ F ] = df_dx(T, state, u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%State variables:
x = state(1);      %north(x coordinate)
y = state(2);      %east (y coordinate)
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
%theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction
omega = state(5);  %angular velocity body fram around z-axis
a = state(6);      %apparent wind velocity
psi = state(7);    %apparent wind angle

%Control variables
delta_r = u(1);     %rudder angle
delta_s = u(2);     %sail angle

%Internal constants                                         original values
p1 = 0.05;          % -         drift coefficient
p2 = .2;            %kg s^-1    tangential friction
p3 = 6000;          %kg m       angular friction
p4 = 1000;          %kg s^-1    sail lift
p5 = 2000;          %kg s^-1    rudder lift                 %2000
p6 = 1;             %m          distance to sail CoE
p7 = 1;             %m          distance to mast
p8 = 2;             %m          distance to rudder
p9 = 300;           %kg         mass of boat
p10 = 10000;        %kg m^2     moment of intertia


%initialize matrix and set elements to zero
F = zeros(7,7);


% x
F(1,1) = 1;
F(1,3) = -T*v*sin(theta);
F(1,4) = T*cos(theta);
F(1,6) = T*p1*cos(psi);
F(1,7) = -T*p1*a*sin(psi);

%y
F(2,2) = 1;
F(2,3) = T*v*cos(theta);
F(2,4) = T*sin(theta);
F(2,6) = T*p1*sin(psi);
F(2,7) = T*p1*a*cos(psi);

%theta
F(3,3) = 1;
F(3,5) = T;

%v
F(4,4) = 1 - T*p2/p9*2*v - .2*T*p5/p9*sin(delta_r)^2;
F(4,6) = T*p4/p9*sin(delta_s)*sin(delta_s-psi);
F(4,7) = -T*p4/p9*a*sin(delta_s)*cos(delta_s-psi);

%omega
F(5,4) = -T*p3/p10*omega - T*p5*p8/p10*cos(delta_r)*sin(delta_r);
F(5,5) = 1 + T*p3/p10*v;
F(5,6) = T*p4/p10*(p6-p7*cos(delta_s))*sin(delta_s-psi);
F(5,7) = -T*p4/p10*(p6-p7*cos(delta_s))*a*cos(delta_s-psi);

%a
F(6,6) = 1;

%psi
F(7,7) = 1;

end







