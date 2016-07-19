function Y = model_sailboat_test(t,X)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

x = X(1);
y = X(2);
psi = X(3);

Y = [t y+1 psi];
end