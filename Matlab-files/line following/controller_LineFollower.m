function [delta_r, delta_sMax, q] = controller_LineFollower(state, W, D, q)
%controller_simpleLine Simple line following controller
%   Controller from the paper "A simple controller for line 
%   following of sailboats" by Luc Jaulin and Fabrice Le Bars

%State variables:
x = state(1);      %north(x coordinate)
y = state(2);      %east (y coordinate)
theta = state(3);  %yaw (angel about d-axis(z), boats heading)
theta = wrapToPi(theta);
v_boat = state(4);      %linear velocity body frame x-direction
%omega = state(5);  %angular velocity body fram around z-axis

psi = W(1);         %true wind angle

a(1) = D(1); %first point
a(2) = D(2);
b(1) = D(3); %second point. Sailing towards second point
b(2) = D(4);

%Constans/Parameters
r = 5;             % m -   cutoff distance
delta_rMax = pi/4;  % rad   maximum rudder angle
gamma = pi/4;       % rad   incidence angle
xi = pi/3;          % rad   close hauled angle


%Step 1
u = 1./hypot(b(1)-a(1),b(2)-a(2)).*[b(1)-a(1) b(2)-a(2)];
v = [x-a(1) y-a(2)];
e = u(1)*v(2)-v(1)*u(2);

%Step 2
if (abs(e) > r/2)
    q = sign(e);
end


%Step 3
phi = atan2((b(2)-a(2)),(b(1)-a(1)));

%Step 4
theta_star = phi-2*gamma/pi*atan(e/r);

%Step 5-9
if ((cos(psi-theta_star)+cos(xi) < 0) || ...
        ((abs(e)<r)&&(cos(psi-phi)+cos(xi)<0)))
    theta_bar = pi + psi - q*xi;
else
    theta_bar = theta_star;
end 
    
%Step 10-11
if cos(theta-theta_bar) >= 0
    delta_r = delta_rMax*sin(theta-theta_bar);
else
    delta_r = delta_rMax*sign(sin(theta-theta_bar));
end
    
%Step 12
delta_sMax = pi/4*(cos(psi-theta_bar)+1);


end

