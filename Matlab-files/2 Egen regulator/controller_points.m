function [delta_r, delta_s, q, k, r_1, beta, e, theta_r, tacking] = controller_points(X, q, W, L, k, r)

% X - boats state variable
% phi - true wind angle
% L - points on line to follow
% q - tack variable
r_1=0,
tacking=q;
theta_r=0;
beta=0;

m(1) = X(1);    %north(x coordinate)
m(2) = X(2);    %east (y coordinate)
theta = X(3);   %yaw (angel about d-axis(z), boats heading)
theta = wrapToPi(theta);
v = X(4);       %linear velocity body frame x-direction


psi = W(1);     %true wind angle
a = W(2);       %true wind velocity

%Apparent wind
%apperent wind speed vector in b-frame
W_ap = [a*cos(psi-theta)-v a*sin(psi-theta)];

psi_ap = atan2(W_ap(2),W_ap(1));    %apperent wind angle in b-frame


%Controller parameters
r = 25;              % m     cutoff distance
delta_rMax = pi/4;  % rad   maximum rudder angle
delta_sMax = pi/3;  % rad   maximum sail angle
delta_sMin = pi/16; % rad   minimum sail angle
gamma = pi/4;       % rad   incidence angle
xi = pi/5;          % rad   no go angle
xi_tack = pi/4;     % rad   tack heading

%line
a(1) = L(2*k+1);    %first point.
a(2) = L(2*k+2);
b(1) = L(2*k+3);    %second point. Sailing along the line from a to b
%towards b
b(2) = L(2*k+4);


% Reference
% if boat is near point b (distance r), navigate towards next point
if hypot(b(1)-m(1),b(2)-m(2)) < r
    k = k + 1;
end


% Controller

%Step 1 - Calculate line angle and distance to line
phi = atan2((b(2)-a(2)),(b(1)-a(1)));       %line angle

u = 1./hypot(b(1)-a(1),b(2)-a(2)).*[b(1)-a(1) b(2)-a(2)];
v = [m(1)-a(1) m(2)-a(2)];
e = u(1)*v(2)-v(1)*u(2);	%distance from boat to line


%Step 2 - Decide navigation type: nominal or tack
% and calculate desired heading
if (abs(wrapToPi(wrapToPi(psi-pi)-phi))<xi) %if true - Tack
    if abs(e)>r/2
        q = sign(e+eps);    %tack variabel. (eps to avoid sign(0), not ideal solution)
    end
    theta_bar = wrapToPi(wrapToPi(psi -pi) - q*xi_tack);
else
    %Nominal saling - sail along line. second term makes the line
    %attractive
    theta_bar = phi-2*gamma/pi*atan(e/r);
end


%Step 3 - calculate rudder angle
%Sign(X_4) flips rudder angle if boat moves backwards
if cos(theta-theta_bar) >= 0    % if not - rudder angle is set to its maxmium
    delta_r = sign(X(4))*delta_rMax*sin(theta-theta_bar);
else
    delta_r = sign(X(4))*delta_rMax*sign(sin(theta-theta_bar));
end

%Step 4 - calculate sail angle
%Linear function of apperant wind to decide delta_s (sheet length)
delta_s = -sign(psi_ap)*((delta_sMin-delta_sMax)/(pi-xi)*abs(psi_ap) + delta_sMax);



end