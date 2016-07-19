close all;clc;
global Wn1c Pn1c Wn1ca Wn1cb M rode_number;
rode_number = 2;

%simulation fixed-free 
origin = [0;0;0];

%b vector of the rode
b_0 = [0 sqrt(2)
       0  0
       -1 -1-sqrt(2)];

r_0 = b_0(:,1)/2.0;

for i=2:length(b_0(1,:))
    r_0 = [r_0 origin+sum(b_0(:,1:i-1),2)+b_0(:,i)/2.0];
end

%creation of q1 and q2
q1 = reshape(r_0,3*rode_number,1);
q2 = reshape(b_0,3*rode_number,1);

%creation of Wn1c

I_blk_n = eye(3);
for i=1:rode_number-1
   I_blk_n=blkdiag(I_blk_n,eye(3));
end

I_blk_n_minus1 = eye(3);
for i=1:rode_number-2
   I_blk_n_minus1=blkdiag(I_blk_n_minus1,eye(3));
end
 I_blk_nm1=I_blk_n_minus1;
 I_blk_n_minus1 = horzcat(I_blk_n_minus1,zeros(length(I_blk_n_minus1(:,1)),3));
 I_blk_n_minus1 = vertcat(zeros(3,length(I_blk_n_minus1(1,:))),I_blk_n_minus1);
 
Wn1ca=-I_blk_n;
Wn1cb=I_blk_n_minus1;

Wn1c = Wn1ca+Wn1cb;
%mass matrix
m =1;
M = m*I_blk_n;
%construction of Pn1c 
p1=vertcat(zeros(3,length(I_blk_nm1(1,:))),I_blk_nm1);
p2=vertcat(I_blk_nm1,zeros(3,length(I_blk_nm1(1,:))));
p=p1+p2;

Pn1c=horzcat(zeros(length(p1(:,1)),3),p);

Nn1c=zeros(3*rode_number,1);
Nn1c(1:3)=-origin;
