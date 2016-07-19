function [ H ] = dh_dx(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%State variables:
%x = state(1);      %north(x coordinate)
%y = state(2);      %east (y coordinate)
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
%theta = wrapToPi(theta);
v = state(4);      %linear velocity body frame x-direction
%omega = state(5);  %angular velocity body fram around z-axis
A = state(6);      %true wind velocity
PSI = state(7);    %true wind angle


%initialize matrix and set elements to zero
H = zeros(6,7);

% x
H(1,1) = 1;
%y
H(2,2) = 1;
%theta
H(3,3) = 1;
%v
H(4,4) = 1;

%a
H(5,3) = -(A*v*sin(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
H(5,4) = (v-A*cos(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
H(5,6) = (A-v*cos(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));
H(5,7) = (A*v*sin(PSI-theta))/(sqrt(A^2-2*A*v*cos(PSI-theta)+v^2));

%psi
H(6,3) = -(A*(A-v*cos(PSI-theta)))/(A^2-2*A*v*cos(PSI-theta)+v^2);
H(6,4) = (A*sin(PSI-theta))/(A^2-2*A*v*cos(PSI-theta)+v^2);
H(6,6) = -(v*sin(PSI-theta))/(A^2-2*A*v*cos(PSI-theta)+v^2);
H(6,7) = (A*(A-v*sin(PSI-theta)))/(A^2-2*A*v*cos(PSI-theta)+v^2);

for i = 5:6
    for j = [3 4 6 7]
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







