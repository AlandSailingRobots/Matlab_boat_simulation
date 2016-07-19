function h_out = h(state)
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

A = state(6);      %true wind velocity
PSI = state(7);    %true wind angle


W_ap = [A*cos(PSI-theta)-v A*sin(PSI-theta)];    


h_out = [x;
         y;
         theta;
         v;
         hypot(W_ap(1),W_ap(2));
         atan2(W_ap(2),W_ap(1))];
  
end