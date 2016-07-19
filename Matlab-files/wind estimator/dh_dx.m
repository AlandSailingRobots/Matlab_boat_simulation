function [ H ] = dh_dx(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%State variables:
A = state(1);      %true wind velocity
PSI = state(2);    %true wind angle
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
%theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction


%initialize matrix and set elements to zero
H = zeros(4,4);


% %a
% H(1,1) = (A-v*cos(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
% H(1,2) = (A*v*sin(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
% H(1,3) = -(A*v*sin(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
% H(1,4) = (v-A*cos(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
% 
% 
% %psi
% H(2,1) = -(v*sin(PSI-theta))/(A^2-2*A*v*cos(PSI-theta)+v^2);
% H(2,2) = (A*(A-v*sin(PSI-theta)))/(A^2-2*A*v*cos(PSI-theta)+v^2);
% H(2,3) = -(A*(A-v*cos(PSI-theta)))/(A^2-2*A*v*cos(PSI-theta)+v^2);
% H(2,4) = (A*sin(PSI-theta))/(A^2-2*A*v*cos(PSI-theta)+v^2);

H(1,1) = 1;
H(2,2) = 1;
H(2,3) = -1;

%theta
H(3,3) = 1;

%v
H(4,4) = 1;


for i = 1:4
    for j = 1:4
        if isnan(H(i,j))
            H(i,j) = 0;
            %NaN
        end
        if isinf(H(i,j))
            H(i,j) = 1000;
            %inf
        end
    end
end


end







