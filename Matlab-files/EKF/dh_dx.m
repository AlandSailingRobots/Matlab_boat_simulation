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
a = state(6);      %apparent wind velocity
psi = state(7);    %apparent wind angle




%initialize matrix and set elements to zero
H = zeros(5,7);


% x
H(1,1) = 1;


%y
H(2,2) = 1;

%theta
H(3,3) = 1;

%a
H(4,3) = -(a*v*sin(psi-theta))/(sqrt(a^2-2*a*v*cos(psi-theta)+v^2));
H(4,4) = (v-a*cos(psi-theta))/(sqrt(a^2-2*a*v*cos(psi-theta)+v^2));
H(4,6) = (a-v*cos(psi-theta))/(sqrt(a^2-2*a*v*cos(psi-theta)+v^2));
H(4,7) = (a*v*sin(psi-theta))/(sqrt(a^2-2*a*v*cos(psi-theta)+v^2));

%psi
H(5,3) = -(a*(a-v*cos(psi-theta)))/(a^2-2*a*v*cos(psi-theta)+v^2);
H(5,4) = (a*sin(psi-theta))/(a^2-2*a*v*cos(psi-theta)+v^2);
H(5,6) = -(v*sin(psi-theta))/(a^2-2*a*v*cos(psi-theta)+v^2);
H(5,7) = (a*(a-v*sin(psi-theta)))/(a^2-2*a*v*cos(psi-theta)+v^2);

for i = 4:5
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







