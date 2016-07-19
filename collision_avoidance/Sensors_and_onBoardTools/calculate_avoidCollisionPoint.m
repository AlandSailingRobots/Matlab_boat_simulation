function avoidCollisionPoint = calculate_avoidCollisionPoint(Z,dim)
    xmin = dim(1);
    xmax = dim(2);
    ymin = dim(3);
    ymax = dim(4);
    [M,I] = min(Z(:));
    [I_row, I_col] = ind2sub(size(Z),I);
    I_x = I_col*(ymax-ymin)/size(Z,2)-(abs(ymin));
    I_y = I_row*(xmax-xmin)/size(Z,1)-(abs(xmin));
    avoidCollisionPoint = [I_x ; I_y];
end   