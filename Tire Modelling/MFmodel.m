function [mu] = MFmodel(SA,SA0);
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
B = SA0(1);
C = SA0(2);
D = SA0(3); 
E = SA0(4);
Sh = SA0(5);
Sv = SA0(6);
mu = D*sind(C*atand(B*(SA+Sh)-E*(B*(SA+Sh)-atand(B*(SA+Sh)))))+Sv;
end

