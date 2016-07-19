function dstate_dt = model_sailboat_3Dof(t,state,u,W)
%model_sailboat_3Dof a model of a sailboat in 3dof
%   Detailed explanation goes here

n = state(1);       %north(x coordinate)
e = state(2);       %east (y coordinate)
psi = state(3);     %yaw (angel about d-axis(z), boats heading)
u = state(4);       %linear velocity body frame x-direction
v = state(5);       %linear velocity body frame y-direction
r = state(6);       %angular velocity body fram around z-axis

%Control variables
delta_r = u(1);     %rudder angle
delta_s = u(2);     %sail angle

%External variables
alpha_aw = W(1);    %apparent wind angle
v_aw = W(2);        %apparent wind speed
                    
%Internal Constants
m =;    %kg mass of boat
I_z = ; %kgs^2 mass moment of inertia

    %distances
    x_r =;    %m    distance from CoG to CoE rudder
    x_m =;    %m    distance from CoG to mast
    x_ms =;   %m    distance from mast to CoE sail





eta = [n e psi]';
nu = [u v r]';

%Transformation matrix. Transforming velocities from b-fram to n-frame
J = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi)  0;
     0        0         1];
 
%system inertia matrix
M = [m-Xud 0     0;
     0     m-Yvd 0;
     0     0     I_z-Nrd];
Minv = inv(M);

%Coriolis-centripal matrix
C = 0; %assume low speed ==> set to zero

%Damping matrix
D

%Restoring forces and moments
g = 0; % no heave, pitch or roll ==> no active forces, set to zero.

%forces and moments from control parameters (sail, rudder)
tau

 
%The model. First order differential equations. 
eta_dot = J * nu;
nu_dot = Minv*tau - Minv*g - Minv*C*nu - Minv*D;
%nu_dot = M\tau - M\g - M\C*nu - M\D;  %alternative (better?) matlab syntax

dstate_dt = [eta_dot; nu_dot];
end