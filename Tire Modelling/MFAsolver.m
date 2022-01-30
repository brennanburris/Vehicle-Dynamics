function [parameterSR] = MFAsolver() 
% Brennan Burris
% brennanburris@gmail.com
% This is a solver that takes tire data inputs to sovle for the Pacejka
% Magic Formula (MF) coefficients B, C, D, E, Sh, and Sv. This solver
% solves for the acceleration characteristics of the tire under braking and
% positive acceleration. 
load('B1654run56');
B = 8;
C = 1.5;
D = 1;
E = 0;
Sh = 0;
Sv = 0;

MuAccel = D*sind(C*atand(B*(SR+Sh)-E*(B*(SR+Sh)-atand(B*(SR+Sh)))))+Sv;

func = @(SR0) mean((NFX-MFmodel(SR,SR0)).^2);
SR0 = [8 1.5 1 0 0 0];
parameterSR = fminunc(func,SR0);
%x(1,3) = x(1,3)/1.75;

% plot(SR, NFX);
% hold on
% SRs = -.25:0.0001:.25;
% Mus = (MFmodel(SRs, parameterSR));
% plot(SRs, Mus)
% xlabel('Slip Ratio');
% ylabel('Longitudinal Force');
% hold on


end