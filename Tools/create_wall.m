function pos=create_wall(a,b,step)
    pos=[];
    slope=(b(2)-a(2))/(b(1)-a(1));
    origin=a(2)-slope*a(1);
    s=norm(b-a)/step;
    for(i=1:step:s)
        pos=[pos [a(1)+i;slope*(a(1)+i)+origin]];
    end
end