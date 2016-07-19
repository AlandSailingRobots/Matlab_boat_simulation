function [ tau2,lambda ] = bar_length_control( Kil,fa,fb,q2,q2dot,L,Lint,Kdl,Ldot,tau1,Ndotdot,Cdot,C,Kdc,Kpc)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global M rode_number;


tau2=zeros(3*rode_number,1);
for i = 1:rode_number
    md = diag(M);
    m =md(1+(i-1)*3); 
    b=q2(1+(i-1)*3:i*3);
    bdot=q2dot(1+(i-1)*3:i*3);
    fbi=fb(:,1+(i-1)*3:i*3);%fa' and fb' orthogonal to b
    fai=fa(:,1+(i-1)*3:i*3);
    tau2i = (6/m)*(fbi-fai)-(b/norm(b))*(Kpl(norm(b)-L)...
        +Kdl*(bdot'*b/norm(b)-Ldot(i)));%+Kil*Lint(i));
    tau2(1+(i-1)*3:i*3) = tau2i;
end

for j = 1:10
    
    lambda = calc_lagrange_mul( tau1,tau2,Ndotdot,Cdot,C,Kdc,Kpc);
    
    if j<10
      for i = 1:rode_number
        md = diag(M);
        m =md(1+(i-1)*3); 
        b=q2(1+(i-1)*3:i*3);
        bdot=q2dot(1+(i-1)*3:i*3);
        fbi=fb(:,1+(i-1)*3:i*3);%fa' and fb' orthogonal to b
        fai=fa(:,1+(i-1)*3:i*3);
        tau2i = (6/m)*(fbi-fai)-(b/norm(b))*(Kpl(norm(b)-L)...
            +Kdl*(bdot'*b/norm(b)-Ldot(i))+Kil*Lint(i));
        tau2(1+(i-1)*3:i*3) = tau2i;
      end
    end
end


end

