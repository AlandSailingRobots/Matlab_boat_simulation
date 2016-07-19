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
A = state(6);      %true wind velocity
PSI = state(7);    %true wind angle

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
p11 = .2;

W_ap = [A.*cos(PSI-theta)-v A.*sin(PSI-theta)];
%apperent wind speed vector in b-frame
psi = atan2(W_ap(:,2),W_ap(:,1));    %apperent wind angle in b-frame
a = hypot(W_ap(:,1),W_ap(:,2));      %apperent wind speed velocity in b-frame


%initialize matrix and set elements to zero
F = zeros(7,7);

a_tw = A;
psi_tw = PSI;
a_aw = a;
psi_aw = psi;

%helping derivatives
daaw_dtheta = -a_tw*v*sin(psi_tw-theta)/sqrt(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
daaw_dv = (v-a_tw*cos(psi_tw-theta))/sqrt(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
dpsiaw_dtheta = -(a_tw*(a_tw-v*cos(psi_tw-theta)))/(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
dpsiaw_dv = a_tw*sin(psi_tw-theta)/(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
daaw_datw = (a_tw-v*cos(psi_tw-theta))/sqrt(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
daaw_dpsitw = (a_tw*v*sin(psi_tw-theta))/sqrt(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
dpsiaw_datw = (-v*sin(psi_tw-theta))/(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);
dpsiaw_dpsitw = a_tw*(a_tw-v*cos(psi_tw-theta))/(a_tw^2-2*a_tw*v*cos(psi_tw-theta)+v^2);

dFs_dtheta = p4*(a_aw*cos(delta_s-psi_aw)*-dpsiaw_dtheta + daaw_dtheta*sin(delta_s-psi_aw));
dFs_dv = p4*(a_aw*cos(delta_s-psi_aw)*-dpsiaw_dv + daaw_dv*sin(delta_s-psi_aw));
dFs_datw = p4*(a_aw*cos(delta_s-psi_aw)*-dpsiaw_datw +daaw_datw*sin(delta_s-psi_aw) );
dFs_dpsitw = p4*(a_aw*cos(delta_s-psi_aw)*-dpsiaw_dpsitw + daaw_dpsitw*sin(delta_s-psi_aw));

dFr_dv = p5*2*v*sin(delta_r);

% x
F(1,1) = 1;
F(1,3) = -T*v*sin(theta);
F(1,4) = T*cos(theta);
F(1,6) = T*p1*cos(psi_tw);
F(1,7) = -T*p1*a_tw*sin(psi_tw);

%y
F(2,2) = 1;
F(2,3) = T*v*cos(theta);
F(2,4) = T*sin(theta);
F(2,6) = T*p1*sin(psi_tw);
F(2,7) = T*p1*a_tw*cos(psi_tw);

%theta
F(3,3) = 1;
F(3,5) = T;

%v
% F(4,4) = 1 - T*p2/p9*2*v - p11*T*p5/p9*sin(delta_r)^2;
% F(4,6) = T*p4/p9*sin(delta_s)*sin(delta_s-psi);
% F(4,7) = -T*p4/p9*a*sin(delta_s)*cos(delta_s-psi);

F(4,3) = T/p9*sin(delta_s)*dFs_dtheta;
F(4,4) = 1 + T/p9*sin(delta_s)*dFs_dv -T/p9*p11*sin(delta_r)*dFr_dv -T*p2/p9*2*v;
F(4,6) = T/p9*sin(delta_s)*dFs_datw;
F(4,7) = -T/p9*sin(delta_s)*dFs_dpsitw;


%omega
%F(5,4) = -T*p3/p10*omega - T*p5*p8/p10*cos(delta_r)*sin(delta_r);
F(5,5) = 1 ;%+ T*p3/p10*v;
%F(5,6) = T*p4/p10*(p6-p7*cos(delta_s))*sin(delta_s-psi);
%F(5,7) = -T*p4/p10*(p6-p7*cos(delta_s))*a*cos(delta_s-psi);

%a
F(6,6) = 1;

%psi
F(7,7) = 1;

end







