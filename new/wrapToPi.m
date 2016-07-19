function [ angle_rad ] = wrapToPi( angle_rad )
%WRAPTOPI Summary of this function goes here
%   Detailed explanation goes here

angle_rad = angle_rad - 2*pi*floor( (angle_rad+pi)/(2*pi) ); 

end