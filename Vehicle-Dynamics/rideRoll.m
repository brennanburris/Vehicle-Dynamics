function [Ksf, Ksr, KpBR, KpBF, Kpb] = rideRoll(omegaf,omegar,Wsf, Wsr, Kt, LRf, LRr, Zf, Zr, hs, as, Ws, RG, T, RL, Wuf, phi, Wt)
%% Ride and roll rates 

%% Front Ride analyis
% Krf = ride rate front, lb/in
% Omegaf = ride frequency front
% Wsf = Sprung weight, front lbs
% hs = cog height, inches
% as = front weight dist
% Ws = sprung weight lbs
% RG = roll gradient deg/g
% Track width
% RLF = static loaded radius in
Tf = T;
Tr = T;

%% Front Ride ANALysis
% front ride rate 
Krf = (4*pi^2*omegaf^2*(Wsf/2))/386.4;

% wheel center rate 
Kwf = (Krf*Kt)/(Kt - Krf);

% front spring rate 
Ksf = Kwf/(LRf^2);

%% REAR Ride ANALysis
% rear ride rate 
Krr = (4*pi^2*omegar^2*(Wsr/2))/386.4;

% wheel center rate 
Kwr = (Krr*Kt)/(Kt - Krr);

% rear spring rate 
Ksr = Kwr/(LRr^2);


%% Roll Rate analysis
% rolling moment lever arm
hrm = hs-(Zf + (Zr-Zf)*(1-as));

% rolling moment per lateral acceleration
mPerA = hrm*Ws/12;

% roll rate
Kp = mPerA/RG;

% front spring roll rate
Kpsf = Krf*(Tf^2)/1375;

% rear spriong roll rate 
Kpsr = Krr*(Tf^2)/1375;

% total available roll rate from springs 
Kps = Kpsf+Kpsr;

% stabilizer bar rate 
Kpb = abs(Kp-Kps);

% total load transfer
TLT = Wt*hs/T;

FLT = as*TLT;

% front roll stiffness 

Kpf = (FLT-((Wsf*Zf)/Tf)-((Wuf*RL)/Tf))*Tf/(12*phi);

% rear bar stiffness
KpBR = abs((Kp-Kpf-Kpsr))/(LRr^2);

% front bar stiffness 
KpBF = abs((Kpf-Kpsf))/(LRf^2);
end

