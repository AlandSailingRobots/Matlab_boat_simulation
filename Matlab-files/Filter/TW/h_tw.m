function h_out = h_tw(state)
%model_sailboat_Jaulin a model of a sailboat in 3dof
%   five states and two control variables. Model from Jaulin (Sailboat 
%   as windmill, Robotic sailing 2013), but simplified handling of sail 
%   and without the "energi of batteris"-state.

%State variables:
x = state(1);      %north(x coordinate)
y = state(2);      %east (y coordinate)
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
%theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction
omega = state(5);  %angular velocity body fram around z-axis

a = state(6);      %apparent wind velocity
phi = state(7);    %apparent wind angle


W_ap = [a*cos(phi-theta)-v a*sin(phi-theta)];    



%differential equations
h_out = [x;
         y;
         theta;
         hypot(W_ap(1),W_ap(2));
         atan2(W_ap(2),W_ap(1))];
  
end