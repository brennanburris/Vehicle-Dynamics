function [parametersSA] = MFCsolver() 
% Brennan Burris
% brennanburris@gmail.com
% This is a solver that takes tire data inputs to sovle for the Pacejka
% Magic Formula (MF) coefficients B, C, D, E, Sh, and Sv. 
load('B1654run9');

B = 8;
C = 1.5;
D = 1;
E = 0;
Sh = 0;
Sv = 0;

func = @(SA0) mean((NFY-MFmodel(SA,SA0)).^2);
SA0 = [8 1.5 1 0 0 0];
parametersSA = fminunc(func,SA0);
% x(1,3) = x(1,3)/2;
plot(SA, NFY);
hold on
SAs = -14:0.1:14;
Mus = (MFmodel(SAs, parametersSA));
plot(SAs, Mus)
xlabel('Slip Angle');
ylabel('Lateral Mu');
hold on

end

