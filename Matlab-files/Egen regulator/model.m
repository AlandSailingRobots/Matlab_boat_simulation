function dstate_dt = model(t,state)
%model_sailboat_Jaulin a model of a sailboat in 3dof
%   five states and two control variables. Model from Jaulin (Sailboat 
%   as windmill, Robotic sailing 2013), but simplified handling of sail 
%   and without the "energi of batteris"-state.

global i q u W D;

%State variables:
x = state(1);      %north(x coordinate)
y = state(2);      %east (y coordinate)
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction
omega = state(5);  %angular velocity body fram around z-axis

%Control variables
%delta_r = u(1,1);     %rudder angle
delta_s = u(1,2);     %sail angle

%External parameters
phi = W(1);         %true wind angle
a = W(2);           %true wind velocity


%Internal constants
p1 = 0.05;          % -         drift coefficient
p2 = .2;            %kg s^-1    tangential friction
p3 = 6000;          %kg m       angular friction
p4 = 1000;          %kg s^-1    sail lift
p5 = 2000;          %kg s^-1    rudder lift
p6 = 1;             %m          distance to sail CoE
p7 = 1;             %m          distance to mast
p8 = 2;             %m          distance to rudder
p9 = 300;           %kg         mass of boat
p10 = 10000;        %kg m^2     mass moment of intertia


%Controller
%Calculate control signals based on current states and destination
[delta_r, delta_s] = controller(state, W, D);

u(i,1) = delta_r;
u(i,2) = delta_s;   %delta_sMax
u(i,3) = q;

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
    (Fs*sin(delta_s)-Fr*sin(delta_r)-sign(v)*p2*v^2)/p9;                 %v_dot
    (Fs*(p6-p7*cos(delta_s))-p8*Fr*cos(delta_r)-p3*omega*v)/p10 ]; %omega_dot
  
i = i + 1;
end