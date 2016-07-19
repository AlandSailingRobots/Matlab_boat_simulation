function dstate_dt = model(~,state,W,U)
%model_sailboat_Jaulin a model of a sailboat in 3dof
%   five states and two control variables. Model from Jaulin (Sailboat 
%   as windmill, Robotic sailing 2013), but simplified handling of sail 
%   and without the "energi of batteris"-state.

%State variables:
%x = state(1);      %north(x coordinate)
%y = state(2);      %east (y coordinate)
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction
omega = state(5);  %angular velocity body fram around z-axis

%Control variables
delta_r = U(1);     %rudder angle
delta_s = U(2);     %sail angle

%External parameters
phi = W(1);         %true wind angle
a = W(2);           %true wind velocity


%Internal constants                                         original values
p1 = 0.05;          % -         drift coefficient           0.05
p2 = 10;            %kg s^-1    tangential friction         0.2
p3 = 6000;          %kg m       angular friction            6000
p4 = 100;          %kg s^-1    sail lift                    1000
p5 = 2000;          %kg s^-1    rudder lift                 2000
p6 = 1;             %m          distance to sail CoE        1
p7 = 1;             %m          distance to mast            1
p8 = 2;             %m          distance to rudder          1
p9 = 300;           %kg         mass of boat                300
p10 = 10000;        %kg m^2     moment of intertia          10 000


%link equations
W_ap = [a*cos(phi-theta)-v a*sin(phi-theta)];    
                                    %apperent wind speed vector in b-frame
phi_ap = atan2(W_ap(2),W_ap(1));    %apperent wind angle in b-frame
a_ap = hypot(W_ap(1),W_ap(2));      %apperent wind speed velocity in b-frame
Fs = p4*a_ap*sin(delta_s-phi_ap);   %Force of wind on sail
Fr = p5*v*sin(delta_r);             %Force of water on rudder

%differential equations
dstate_dt = [...
    v*cos(theta) + p1*a*cos(phi);       %x_dot
    v*sin(theta) + p1*a*sin(phi);       %y_dot
    omega;                              %theta_dot
    %(Fs*sin(delta_s)-Fr*sin(delta_r)-sign(v)*p2*v^2)/p9;                 %v_dot
    (Fs*sin(delta_s)-.2*Fr*sin(delta_r)-sign(v)*p2*v^2)/p9; 
    (Fs*(p6-p7*cos(delta_s))-p8*Fr*cos(delta_r)-p3*omega*v)/p10 ]; %omega_dot
  
end