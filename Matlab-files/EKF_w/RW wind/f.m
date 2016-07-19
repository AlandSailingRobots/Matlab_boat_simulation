function next_state = f(T,state)
%model_sailboat_Jaulin a model of a sailboat in 3dof
%   five states and two control variables. Model from Jaulin (Sailboat 
%   as windmill, Robotic sailing 2013), but simplified handling of sail 
%   and without the "energi of batteris"-state.

%State variables:
A = state(1);      %true wind velocity
PSI = state(2);    %true wind angle
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
v = state(4);      %linear velocity body frame x-direction



%differential equations
next_state = [A;
              PSI;
              theta;
              v];
  
end