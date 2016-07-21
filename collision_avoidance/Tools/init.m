function init
    close all; clear vars; clc;
    W = get(0,'MonitorPositions'); % plein ecran
    if(size(W,1)<2)
        figure('Position',[0 0 W(3) W(4)]);
    else
        figure('Position',[W(2,1) 0 W(2,3)-W(2,1) W(2,4)]);
    end
    set(gca,'FontSize',12);
    %axis[xmin xmax ymin ymax];
    axis normal
    %f = getframe;              %Capture screen shot
    %[im,map] = frame2im(f);    %Return associated image data
    %imwrite(im,'image.bmp');
end  