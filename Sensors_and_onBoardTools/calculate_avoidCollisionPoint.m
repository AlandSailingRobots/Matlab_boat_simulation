function avoidCollisionPoint = calculate_avoidCollisionPoint(Z,dim)
    xmin = dim(1);
    xmax = dim(2);
    ymin = dim(3);
    ymax = dim(4);
    [M,I] = min(Z(:));
    [I_row, I_col] = ind2sub(size(Z),I);
    I_x = (I_col-1)/size(Z,2)*(xmax-xmin)+xmin; % ((ymax-ymin)/(xmax-xmin))*
    I_y = (I_row-1)/size(Z,1)*(ymax-ymin)+ymin; % ((xmax-xmin)/(ymax-ymin))*
    avoidCollisionPoint = [I_x ; I_y];
end   