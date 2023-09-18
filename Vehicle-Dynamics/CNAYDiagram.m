%% CN - AY Diagram
% Yaw moment vs Y acceleration
% Brennan Burris
% brennanburris@gmail.com
% 9/04/2023
% You will need to download TM.m to use this script
%
% This script outputs the theoretical limit of the tire based on the corner
% radius. It can be used to predict handling at the limit of grip.

%% 
clear
clc
close all

%% Coordinate System
% X -> + Forward
% Y -> + Drivers Right
% Z -> + Up

% angles -> positive CCW
% path to heading is slip angle

%% Variables and Vehicle Specifications
W = 600; % Weight in lbs
hF = 11.25; % Static Front COG height in inches
hR = 11.25; % Static Rear COG height in inches
l = 60.5; % Wheelbase in inches
t = 48; % Track width in inches
Wdist = 0.43; % Factor of static weight over front axle
downForce = 150; % Total downforce in lbs at 40 mph
aeroBal = 0.45; % Front aero factor 
kF = 300; % Front Spring Rate in lbs/in
kR = 300; % Rear Spring Rate in lbs/in
camF = -2; % Front static camber in degrees
camR = -2; % Rear static camber in degrees
CCf = -0.88; % Front Camber compensation in degrees camber gain / degree body roll
CCr = -0.88; % Rear Camber compensation in degrees camber gain / degree body roll
camBumpf = -0.6; % Degrees of camber gain/inch bump front
camBumpr = -0.6; % Degrees of camber gain/inch bump rear
LLTDf = 0.6; % Lateral Load Transfer Distribution Factor Front
toeLF = 0.5; % Left front toe angle in degrees
toeRF = -1; % Right front toe angle in degrees
toeLR = -1; % Left rear toe angle in degrees
toeRR = 0.5; % Right rear toe angle in degrees
rollGrad = 1; % Body roll in deg/G
R = 30; % Corner radius in feet
ack = -.08; % Ackermann percentage factor
V = 40; % Velocity guess in miler per hour

%% Dependent Variable Calculations
a = 34.53; % Estimated distance from COG to front track in inches
b = l-a; % Estimated distance from COG to rear track in inches
h = (hF+hR)/2; % Average COG height
R = R*12; % Radius from feet to inches
LLTDr = 1-LLTDf;
FD = downForce*aeroBal; % Front downforce
RD = downForce*(1-aeroBal); % Rear downforce
V = V*5280/3600; % Convert to fps

%% Initialization
delta = -8:8; % Theoretical steering input
beta = -8:8; % Theoretical sideslip
Ay =  zeros(length(delta),length(beta)); % Initial Ay guess
SALF= zeros(length(delta),length(beta));
SARF = zeros(length(delta),length(beta));
SALR = zeros(length(delta),length(beta));
SARR = zeros(length(delta),length(beta));
LLT = zeros(length(delta),length(beta));
LLTf= zeros(length(delta),length(beta));
LLTr = zeros(length(delta),length(beta));
FZLF = zeros(length(delta),length(beta));
FZRF = zeros(length(delta),length(beta));
FZLR = zeros(length(delta),length(beta));
FZRR = zeros(length(delta),length(beta));
IALF = zeros(length(delta),length(beta));
IARF = zeros(length(delta),length(beta));
IALR = zeros(length(delta),length(beta));
IARR = zeros(length(delta),length(beta));
NFXlf = zeros(length(delta),length(beta));
FXlf = zeros(length(delta),length(beta));
NFYlf = zeros(length(delta),length(beta));
FYlf = zeros(length(delta),length(beta));
MZlf = zeros(length(delta),length(beta));
PTlf = zeros(length(delta),length(beta));
NFXrf = zeros(length(delta),length(beta));
FXrf = zeros(length(delta),length(beta));
NFYrf = zeros(length(delta),length(beta));
FYrf = zeros(length(delta),length(beta));
MZrf = zeros(length(delta),length(beta));
PTrf = zeros(length(delta),length(beta));
NFXlr = zeros(length(delta),length(beta));
FXlr = zeros(length(delta),length(beta));
NFYlr = zeros(length(delta),length(beta));
FYlr = zeros(length(delta),length(beta));
MZlr = zeros(length(delta),length(beta));
PTlr = zeros(length(delta),length(beta));
NFXrr = zeros(length(delta),length(beta));
FXrr = zeros(length(delta),length(beta));
NFYrr = zeros(length(delta),length(beta));
FYrr = zeros(length(delta),length(beta));
MZrr = zeros(length(delta),length(beta));
PTrr = zeros(length(delta),length(beta));
CN = zeros(length(delta),length(beta));

%% CN AY Calculations
for i = 1:length(delta) % Steering input in degrees
    for j = 1:length(beta)% Sideslip in degrees
        
        % Slip angle calculations
        SALF(i,j) = ((beta(j)*pi/180) - (delta(i)*pi/180) + (toeLF*pi/180))*180/pi; % Left front slip angle in degrees
        SARF(i,j) = ((beta(j)*pi/180) - (delta(i)*pi/180) + (toeRF*pi/180))*180/pi; % Right front slip angle in degrees
        SALR(i,j) = ((beta(j)*pi/180) + (toeLR*pi/180))*180/pi; % Left rear slip angle in degrees
        SARR(i,j) = ((beta(j)*pi/180) + (toeRR*pi/180))*180/pi; % Right rear slip angle in degrees
        
        % Fixed point iterative solution
        for k = 1:3
            
            % Lateral Load Transfer Calculation
            LLT(i,j) = W*Ay(i,j)*h/t; % Negative if LH manuevre
            LLTf(i,j)= LLT(i,j)*LLTDf; % Front Load Transfer
            LLTr(i,j) = LLT(i,j)*LLTDr; % Rear Load Transfer
            
            FZLF(i,j) = W*Wdist/2 + LLTf(i,j) + FD/2; % Normal force LF tire, lbs. decreases if LH turn (+ -LLT)
            FZRF(i,j) = W*Wdist/2 - LLTf(i,j) + FD/2; % Normal force RF tire, lbs. increases if LH turn (- - LLT)
            FZLR(i,j) = W*(1-Wdist)/2 + LLTr(i,j) + RD/2; % Normal force LR tire, lbs. decreases if LH turn (+ - LLT)
            FZRR(i,j) = W*(1-Wdist)/2 - LLTr(i,j) + RD/2; % Normal force RR tire, lbs. increases if LH turn (- - LLT)
            
            % Check number one complete here - evaluates correctly
            
            % Camber Calculations
            IALF(i,j) = -1*(rollGrad*Ay(i,j)*CCf) + camF; % IA LF gains negative if LH maneuvre (-Ay)
            IARF(i,j) = (rollGrad*Ay(i,j)*CCf) + camF; % IA RF gains positive if LH maneuvre (-Ay)
            IALR(i,j) = -1*(rollGrad*Ay(i,j)*CCr) + camR; % IA LR gains negative if LH maneuvre (-Ay)
            IARR(i,j) = (rollGrad*Ay(i,j)*CCr) + camR; % IA RR gains positive if LH maneuvre (-Ay)
            
            % Check number 2 complete here - evaluates correctly
            
            % Tire Calculations
            [NFXlf(i,j), FXlf(i,j), NFYlf(i,j), FYlf(i,j), MZlf(i,j), PTlf(i,j)] = TM(FZLF(i,j), SALF(i,j), 0, IALF(i,j)); % Left front tire calculations
            [NFXrf(i,j), FXrf(i,j), NFYrf(i,j), FYrf(i,j), MZrf(i,j), PTrf(i,j)] = TM(FZRF(i,j), SARF(i,j), 0, IARF(i,j)); % Right front tire calculations
            [NFXlr(i,j), FXlr(i,j), NFYlr(i,j), FYlr(i,j), MZlr(i,j), PTlr(i,j)] = TM(FZLR(i,j), SALR(i,j), 0, IALR(i,j)); % Left rear tire calculations
            [NFXrr(i,j), FXrr(i,j), NFYrr(i,j), FYrr(i,j), MZrr(i,j), PTrr(i,j)] = TM(FZRR(i,j), SARR(i,j), 0, IARR(i,j)); % Right rear tire calculations
            
            % Sum of forces
            Ay(i,j) = (FYlf(i,j)+FYrf(i,j)+FYlr(i,j)+FYrr(i,j))/W;
            
            % Yaw Moment Coefficient
            CN(i,j) = (((FYlf(i,j) + FYrf(i,j))*a) - ((FYlr(i,j) + FYrr(i,j))*b))/12;
        end
    end
end

AYl = min(Ay);
Ayr = max(Ay);
%% Plotting, scheming, conspiring, and general mischevious thought processes
% Shut up

[M, N] = size(CN);

for i = 1:M
    
    plot(Ay(:,i), CN(:,i), 'r')
    hold on
    % This plots the moments generated by steering the car
end

for i = 1:N
    plot(Ay(i,:), CN(i,:), 'b')
    hold on
    % This plots the moments generated by sliding the car
end

yline(0)
xline(0)
grid on
title('Yaw Moment vs Lateral Acceleration')
xlabel('Lateral Acceleration (G)')
ylabel('Yaw Moment (lbs*ft)')

figure
plot(Ay,FZLF)
hold on
plot(Ay, FZRF)
hold on
plot(Ay,FZLR)
hold on
plot(Ay, FZRR)
hold off 