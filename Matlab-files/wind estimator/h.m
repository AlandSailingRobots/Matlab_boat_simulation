function h_out = h(state)
%model_sailboat_Jaulin a model of a sailboat in 3dof
%   five states and two control variables. Model from Jaulin (Sailboat 
%   as windmill, Robotic sailing 2013), but simplified handling of sail 
%   and without the "energi of batteris"-state.

%State variables:
A = state(1);      %true wind velocity
PSI = state(2);    %true wind angle
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
%theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction


% W_ap = [A*cos(PSI-theta)-v A*sin(PSI-theta)];    
% 
% 
% h_out = [hypot(W_ap(1),W_ap(2));
%          atan2(W_ap(2),W_ap(1));
%          theta;
%          v];

h_out = [A;
         PSI-theta;
         theta;
         v];
     
% 
% W_world = [A*cos(PSI)-v*cos(theta) A*sin(PSI)-v*sin(theta)];
%      
%  h_out = [hypot(W_world(1),W_world(2));
%           atan2(W_world(2),W_world(1));
%           theta;
%           v];
%   
end