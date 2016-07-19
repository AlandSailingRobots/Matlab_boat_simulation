



A = 4;
PSI = pi/3;
theta = pi/4;
v = 2;
[A,PSI]

W_ap = [A*cos(PSI-theta)-v A*sin(PSI-theta)];    
a = hypot(W_ap(1),W_ap(2));
psi = atan2(W_ap(2),W_ap(1));
[a, psi]

W_t = [a*cos(psi)+v a*sin(psi)];  
theta = -theta;
W_t2 = [cos(theta) sin(theta); -sin(theta) cos(theta)]*W_t';

A2 = hypot(W_t2(1),W_t2(2));
PSI2 = atan2(W_t2(2),W_t2(1));
[A2,PSI2]