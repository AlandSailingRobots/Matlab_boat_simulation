clear all
%Apparent wind

%Båt riktad norrut. true wind alltid 1 m/s
%plot 1 v = 0;
%plot 2 v = 2 m/s
%wind börjar


a = 1;
psi = -pi:.3:pi;    %true wind
theta = 0;
v = 0;

                                %apperent wind speed vector in b-frame
W_apX = a.*cos(psi-theta)+v;
W_apY = a.*sin(psi-theta);    
                                    
psi_ap = atan2(W_apY,W_apX);    %apperent wind angle in b-frame


figure(1)
plot(psi, psi_ap)

a = 1;
psi = -pi:.3:pi;    %true wind
theta = 0;
v = 1;

                                %apperent wind speed vector in b-frame
W_apX = a.*cos(psi-theta)-v;
W_apY = a.*sin(psi-theta);    
                                    
psi_ap = atan2(W_apY,W_apX);    %apperent wind angle in b-frame


figure(2)
plot(psi, psi_ap)
axis([-pi pi -pi pi].*1.1)