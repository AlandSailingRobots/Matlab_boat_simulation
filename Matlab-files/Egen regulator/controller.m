function [delta_r, delta_s] = controller(X, W, L)

global q;

% X - boats state variable
% phi - true wind angle
% L - points on line to follow
% q - tack variable

m(1) = X(1);      %north(x coordinate)
m(2) = X(2);      %east (y coordinate)
theta = X(3);  %yaw (angel about d-axis(z), boats heading)
theta = wrapToPi(theta);
v = X(4);      %linear velocity body frame x-direction
omega = X(5);  %angular velocity body fram around z-axis

psi = W(1);         %true wind angle
a = W(2);           %true wind velocity

W_ap = [a*cos(psi-theta)-v a*sin(psi-theta)];    
                                    %apperent wind speed vector in b-frame
psi_ap = atan2(W_ap(2),W_ap(1));    %apperent wind angle in b-frame


%Controller parameters
r = 5;              % m     cutoff distance
delta_rMax = pi/4;  % rad   maximum rudder angle
delta_sMax = pi/2;  % rad   maximum sail angle
delta_sMin = pi/16; % rad   minimum sail angle
gamma = pi/4;       % rad   incidence angle
xi = pi/4;          % rad   no go angle

%line
a(1) = L(1); %first point
a(2) = L(2);
b(1) = L(3); %second point. Sailing towards second point
b(2) = L(4);


%Step 1 - Calculate line angle and distance to line
phi = atan2((b(2)-a(2)),(b(1)-a(1)));

u = 1./hypot(b(1)-a(1),b(2)-a(2)).*[b(1)-a(1) b(2)-a(2)];
v = [m(1)-a(1) m(2)-a(2)];
e = u(1)*v(2)-v(1)*u(2);


%Step 2 - Decide navigation type: nominal or tack
if (abs(psi_ap)>pi-xi) %if true - Tack
    if abs(e)>r/2
        q = sign(e+eps);
    end
    theta_bar = wrapToPi(phi - q*pi/2);
else 
    %Nominal saling
    theta_bar = phi; 
end

%Step 3 - Update desired heading to become something that is attracting to
%line...
theta_bar = theta_bar-2*gamma/pi*atan(e/r);



%Step 4 - calculate rudder angle
if cos(theta-theta_bar) >= 0
    delta_r = delta_rMax*sin(theta-theta_bar);
else
    delta_r = delta_rMax*sign(sin(theta-theta_bar));
end

%Step 5 - calculate sail angle
delta_s = -sign(psi_ap)*((delta_sMin-delta_sMax)/(pi-xi)*psi_ap + delta_sMax);






end