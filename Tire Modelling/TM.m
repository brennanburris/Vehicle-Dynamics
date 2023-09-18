function [NFX, FX, NFY, FY, MZ, PT] = TM(FZ, SA, SR, IA)
%% THE Tire Model
%% Brennan Burris
% 7/9/2023
% brennanburris@gmail.com
%% Definitions
% SA = Slip angle in degrees - Positive (CCW) Slip for LH Turn
% SR = Slip ratio in degrees
% FZ = normal force in lbs
% IA = camber angle in degrees
% NFX = Friction Coefficient in the X direction
% NFY = Friction Coefficient in the Y direction
% MZ = Self Aligning Moment in lbs*in
% PT = pneumatic trail in in
%% Description
% This is the result of a lot of hard work. It includes pure slip equations
% for longitudinal and lateral acceleration, as well as the combined slip
% equations.

% Each equation represents tires at different inclination angles and
% different normal forces. Based on the inputs of FZ and IA, this model
% will output the force the tire can create given these parameters.

% All this model does is interpolate between values. First it calculates
% the equations for tire force, then it interpolates between. Cake.

% Inputs are IA, FZ, SA, and SR
% The first number after NFY denotes the inclination angle. The first
% letter, either p or c denotes pure slip (p) or combined slip (c).
% Therefore, NFY0p would stand for NFY at IA = 0 in pure slip under
% cornering. NFX4c would stand for NFX at IA = 4 under combined slip.

%% Updates
% 1/31/2022
% Added magnitude coefficient (d) extrapolator for high FZ
% Added warning if IA > 4 degrees
% TM no longer extrapolates from greater than 4 degrees it is too hard :(

% 2/3/2023
% Changed knockdown factor

% 7/8/2023
% Added MZ

% 7/9/2023
% Corrected max grip level for increasing load

% 7/12/2023
% Corrected grip levels for increasing load

% 8/07/2023
% Removed MZ equations and instead use PT curve to find MZ

% 8/15/2023
% Corrected sign of output force - negative for left turn, positive for
% right turn

% 8/20/2023
% Corrected location of maximum Lateral mu

% 9/08/2023
% made -SA correlate with positive mu. A NEGATIVE slip angle correlates
% with a LEFT turn 

%% Governing Equation
% NFY = D.*sind(C.*atand(B.*(SA+...
%  Sh)-E.*(B.*(SA+sh)...
%  -atand(B.*(SA+Sh)))))+Sv

%% Housekeeping
if nargin ~= 4
    error('Must have 4 inputs you blockhead')
end

close all

FZcorrect = 0;
FZo = FZ;


if abs(FZ) > 250
    FZ = -250;
    FZcorrect = 1;
end

FZ = -1*abs(FZ);
IA = abs(IA);

if SA < 0 || SR < 0
    correction = 0;
else
    correction = 1;
end

if IA > 4
    IA = 4;
    warning('Inclination angle is greater than 4, what the fuck? Are you trying to break my shit? Operating with IA = 4 degrees you tool')
end


for index = 1:2
    %% Pure slip equations
    % Layout is as follows:
    % [IA = 0,2, or 4 FZ = 50;
    %  IA = 0,2, or 4 FZ = 150;
    %  IA = 0,2, or 4 FZ = 250]
    
    %% Definitions
    % CCP = Coefficients Cornering Pure - Pure Cornering Coefficients
    % CLP = Coefficients Longitudinal Pure - Pure Longitudinal Coefficients
    % CCC = Coefficients Cornering Combined
    % CLC = Coefficients Longitudinal Combined
    
    IA = abs(IA);
    
    if index == 1
        SA = abs(SA);
        SR = abs(SR);
    else
        SA = -1*abs(SA);
        SR = -1*abs(SR);
    end
    
    % Pure Cornering Coefficients
     coefficientsCornering0 = [0.161192064,1.30000003*1.1, 2.535783706, 0.0119*3, 0,0;...
        0.108342744, 1.500000043*1.1,	2.4, 0.009399707*3, 0, 0; ...
        0.06, 1.831271806*1.1,	2.25, .01*3, 0, 0];  % Fixed
    
    coefficientsCornering2 = [0.119333911373298,1.50000002832375,2.41325957351056,0.0122501549335187, 0,0;...
        0.100637701307224,1.60000015502581,2.36789891734450,0.00979987631457392,0,0;...
        0.0708463663938786,2.49997849218144,2.30027088430659,2.64782017943538e-07,0,0]; % Fixed
    
    coefficientsCornering4 = [0.119582739537369,1.45000018076451,2.30330464551135,0.0111695362812682*3, 0,0;...
        0.00178965252122583,1.49981859188847,2.24951129591912,1.66587891745016*1.5,0,0;...
        0.00564366644391225,2.08395746853433,2.2,0.252788418300819*1.5,0,0]; % Fixed
    
    coefficientsLong0 = [8.49950959059766,1.75000032410059,-2.03579172370720,0.0138948889989628,0,0;...
        8.49960000000000,1.57800000000000,-2.30000000000000,0.0156000000000000,0,0;...
        4.61261975097272,1.70106061786626,-2.29188461972766,0.0354761676530278,0,0]; % Fixed
    
    coefficientsLong2 = [8.49999230950463,1.50000011465249,-1.98468235181166,0.0247640721207096,0,0;...
        8.49926901869931,1.94736190886371,-2.36024392333249,0.00690956126379126,0,0;...
        8.22147767172531,1.75001394244138,-2.33318518258399,0.0127707325439920,0,0];
    
    coefficientsLong4 = [8.05987216361744,1.75007806742004,-1.98866565572401,0.0136204759577897,0,0;...
        7.49297502182196,1.75000201295265,-2.27923555906740,0.0165838853317732,0,0;...
        6.96595101484201,1.74992596179042,-2.61226150246255,0.0201920442097409,0,0]; % Fixed
    
    % combined grip coefficients
    ccc0 = [1.01522681545190,1.40766930476357,0.249657608214171,0.0472634056656552,0.0500000000000000;42.2603420252637,12.9215250701272,67.3303393841706,-2.12060453806994,0.0500000000000000;-0.501655372725928,5.82093689196927,-0.240050993913462,-0.0644980569124586,0.0500000000000000];
    
    ccc2 = [5.48088380641448,1.04840611075299,0.0156952514909401,0.0339816789928007,0.0500000000000000;4.61847089280628,1.13652767125759,0.0145244867962192,0.0328134865032152,0.0500000000000000;1.66547655602971,1.53459808601567,0.0663795332851410,0.0557409180810074,0.0500000000000000];
    
    ccc4 = [-2.80544203664387,-4998.99999786752,6211.29934674600,9458.06017985634,2522.35372073046;3.09790219548835,0.981994881885186,0.0338296008062256,0.00762296760851838,0.0500000000000000;0.961310984668721,1.15150495869317,0.267064852030244,0.0379587998792334,0.0500000000000000];
    
    ccl0 = [1.01522681545190,1.40766930476357,0.249657608214171,0.0472634056656552,0.0500000000000000;42.2603420252637,12.9215250701272,67.3303393841706,-2.12060453806994,0.0500000000000000;-0.501655372725928,5.82093689196927,-0.240050993913462,-0.0644980569124586,0.0500000000000000];
    
    ccl2 = [5.48088380641448,1.04840611075299,0.0156952514909401,0.0339816789928007,0.0500000000000000;4.61847089280628,1.13652767125759,0.0145244867962192,0.0328134865032152,0.0500000000000000;1.66547655602971,1.53459808601567,0.0663795332851410,0.0557409180810074,0.0500000000000000];
    
    ccl4 = [-2.80544203664387,-4998.99999786752,6211.29934674600,9458.06017985634,2522.35372073046;3.09790219548835,0.981994881885186,0.0338296008062256,0.00762296760851838,0.0500000000000000;0.961310984668721,1.15150495869317,0.267064852030244,0.0379587998792334,0.0500000000000000];
    
    
    % MZ Coefficients
    
    %% Pure Cornering
    if SA ~= 0
        %% IA = 0 NFY Pure cornering Equations
        NFY050p = @(SA) coefficientsCornering0(1,3).*sind(coefficientsCornering0...
            (1,2).*atand(coefficientsCornering0(1,1).*(SA+...
            coefficientsCornering0(1,5))-coefficientsCornering0(1,4).*...
            (coefficientsCornering0(1,1).*(SA+coefficientsCornering0(1,5))...
            -atand(coefficientsCornering0(1,1).*(SA+coefficientsCornering0...
            (1,5))))))+coefficientsCornering0(1,6);
        
        NFY0150p = @(SA) coefficientsCornering0(2,3).*sind(coefficientsCornering0...
            (2,2).*atand(coefficientsCornering0(2,1).*(SA+...
            coefficientsCornering0(2,5))-coefficientsCornering0(2,4).*...
            (coefficientsCornering0(2,1).*(SA+coefficientsCornering0(2,5))...
            -atand(coefficientsCornering0(2,1).*(SA+coefficientsCornering0...
            (2,5))))))+coefficientsCornering0(2,6);
        
        NFY0250p =  @(SA) coefficientsCornering0(3,3).*sind(coefficientsCornering0...
            (3,2).*atand(coefficientsCornering0(3,1).*(SA+...
            coefficientsCornering0(3,5))-coefficientsCornering0(3,4).*...
            (coefficientsCornering0(3,1).*(SA+coefficientsCornering0(3,5))...
            -atand(coefficientsCornering0(3,1).*(SA+coefficientsCornering0...
            (3,5))))))+coefficientsCornering0(3,6);
        
        
        %% IA = 2 NFY Pure cornering Equations
        NFY250p = @(SA) coefficientsCornering2(1,3).*sind(coefficientsCornering2...
            (1,2).*atand(coefficientsCornering2(1,1).*(SA+...
            coefficientsCornering2(1,5))-coefficientsCornering2(1,4).*...
            (coefficientsCornering2(1,1).*(SA+coefficientsCornering2(1,5))...
            -atand(coefficientsCornering2(1,1).*(SA+coefficientsCornering2...
            (1,5))))))+coefficientsCornering2(1,6);
        
        NFY2150p = @(SA) coefficientsCornering2(2,3).*sind(coefficientsCornering2...
            (2,2).*atand(coefficientsCornering2(2,1).*(SA+...
            coefficientsCornering2(2,5))-coefficientsCornering2(2,4).*...
            (coefficientsCornering2(2,1).*(SA+coefficientsCornering2(2,5))...
            -atand(coefficientsCornering2(2,1).*(SA+coefficientsCornering2...
            (2,5))))))+coefficientsCornering2(2,6);
        
        NFY2250p = @(SA) coefficientsCornering2(3,3).*sind(coefficientsCornering2...
            (3,2).*atand(coefficientsCornering2(3,1).*(SA+...
            coefficientsCornering2(3,5))-coefficientsCornering2(3,4).*...
            (coefficientsCornering2(3,1).*(SA+coefficientsCornering2(3,5))...
            -atand(coefficientsCornering2(3,1).*(SA+coefficientsCornering2...
            (3,5))))))+coefficientsCornering2(3,6);
        
        
        %% IA = 4 Pure cornering matrix
        NFY450p = @(SA) coefficientsCornering4(1,3).*sind(coefficientsCornering4...
            (1,2).*atand(coefficientsCornering4(1,1).*(SA+...
            coefficientsCornering4(1,5))-coefficientsCornering4(1,4).*...
            (coefficientsCornering4(1,1).*(SA+coefficientsCornering4(1,5))...
            -atand(coefficientsCornering4(1,1).*(SA+coefficientsCornering4...
            (1,5))))))+coefficientsCornering4(1,6);
        
        NFY4150p = @(SA) coefficientsCornering4(2,3).*sind(coefficientsCornering4...
            (2,2).*atand(coefficientsCornering4(2,1).*(SA+...
            coefficientsCornering4(2,5))-coefficientsCornering4(2,4).*...
            (coefficientsCornering4(2,1).*(SA+coefficientsCornering4(2,5))...
            -atand(coefficientsCornering4(2,1).*(SA+coefficientsCornering4...
            (2,5))))))+coefficientsCornering4(2,6);
        
        NFY4250p = @(SA) coefficientsCornering4(3,3).*sind(coefficientsCornering4...
            (3,2).*atand(coefficientsCornering4(3,1).*(SA+...
            coefficientsCornering4(3,5))-coefficientsCornering4(3,4).*...
            (coefficientsCornering4(3,1).*(SA+coefficientsCornering4(3,5))...
            -atand(coefficientsCornering4(3,1).*(SA+coefficientsCornering4...
            (3,5))))))+coefficientsCornering4(3,6);
        
        
        %% Interpolation between IA and FZ values
        
        % IA between 0 and 2 deg and FZ between 50 and 150 lbs
        if (IA >= 0 && IA < 2) && (FZ <= -50 && FZ >= -150)
            NFY1 = interp1([0,2], [NFY050p(SA), NFY250p(SA)], IA);
            NFY2 = interp1([0,2], [NFY0150p(SA), NFY2150p(SA)], IA);
            NFY = interp1([-50, -150], [NFY1, NFY2], FZ);
            % IA between 2 and 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -50 && FZ >= -150)
            NFY1 = interp1([2,4], [NFY250p(SA), NFY450p(SA)], IA);
            NFY2 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA);
            NFY = interp1([-50, -150], [NFY1, NFY2], FZ);
            
            % IA above 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 4) && (FZ <= -50 && FZ >= -150)
            NFY = interp1([-50,-150], [NFY450p(SA), NFY4150p(SA)], FZ);
            
            
            % IA between 0 and 2 deg and FZ between 150 and 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -150 && FZ >= -250)
            NFY1 = interp1([0,2], [NFY0150p(SA), NFY2150p(SA)], IA);
            NFY2 = interp1([0,2], [NFY0250p(SA), NFY2250p(SA)], IA);
            NFY = interp1([-150, -250], [NFY1, NFY2], FZ);
            
            % IA between 2 and 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -150 && FZ >= -250)
            NFY1 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA);
            NFY2 = interp1([2,4], [NFY2250p(SA), NFY4250p(SA)], IA);
            NFY = interp1([-150, -250], [NFY1, NFY2], FZ);
            
            % IA above 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 4) && (FZ <= -150 && FZ >= -250)
            NFY = interp1([-150,-250], [NFY4150p(SA), NFY4250p(SA)], FZ);
            
            
%             % IA between 0 and 2 deg and FZ higher than 250 lbs
%         elseif (IA >= 0 && IA < 2) && (FZ < -250)
%             NFY = interp1([0,2], [NFY0Hp(SA), NFY2Hp(SA)], IA);
%             
%             % IA between 2 and 4 deg and FZ above 250 lbs
%         elseif (IA >= 2 && IA < 4) && (FZ < -250)
%             NFY = interp1([2,4], [NFY2Hp(SA), NFY4Hp(SA)], IA);
%             
%             % IA above 4 deg and FZ above 250 lbs
%         elseif (IA >= 4) && (FZ < -250)
%             NFY = interp1([-250, FZ], [NFY4250p(SA), NFY4Hp(SA)], FZ);
            
            % IA between 0 and 2 deg and FZ below 50 lbs
        elseif(IA >= 0 && IA < 2) && (FZ >= -50)
            NFY1 = interp1([0,2], [NFY050p(SA), NFY250p(SA)], IA);
            NFY2 = interp1([0,2], [NFY0150p(SA), NFY2150p(SA)], IA);
            NFY = interp1([-50, -150], [NFY1, NFY2], FZ, 'linear', 'extrap');
            
            % IA between 2 and 4 deg and FZ below 50 lbs
        elseif (IA >= 2 && IA < 4) && (FZ >= -50)
            NFY1 = interp1([2,4], [NFY250p(SA), NFY450p(SA)], IA);
            NFY2 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA);
            NFY = interp1([-50, -150], [NFY1, NFY2], FZ, 'linear', 'extrap');
            
            % IA above 4 deg and FZ below 50 lbs
        elseif (IA >= 4) && (FZ > -50)
            NFY = interp1([-50, -150], [NFY450p(SA), NFY4150p(SA)], IA, 'linear', 'extrap');
            
        else
            disp(FZ)
            disp(IA)
            error('not all conditions accounted for')
        end
    end
    
    %% Pure Acceleration
    % IA = 0 Pure longitudinal matrix
    
    if SR ~= 0
        NFX050p = @(SR) coefficientsLong0(1,3).*sind(coefficientsLong0...
            (1,2).*atand(coefficientsLong0(1,1).*(SR+...
            coefficientsLong0(1,5))-coefficientsLong0(1,4).*...
            (coefficientsLong0(1,1).*(SR+coefficientsLong0(1,5))...
            -atand(coefficientsLong0(1,1).*(SR+coefficientsLong0...
            (1,5))))))+coefficientsLong0(1,6);
        
        NFX0150p = @(SR) coefficientsLong0(2,3).*sind(coefficientsLong0...
            (2,2).*atand(coefficientsLong0(2,1).*(SR+...
            coefficientsLong0(2,5))-coefficientsLong0(2,4).*...
            (coefficientsLong0(2,1).*(SR+coefficientsLong0(2,5))...
            -atand(coefficientsLong0(2,1).*(SR+coefficientsLong0...
            (2,5))))))+coefficientsLong0(2,6);
        
        NFX0250p = @(SR) coefficientsLong0(3,3).*sind(coefficientsLong0...
            (3,2).*atand(coefficientsLong0(3,1).*(SR+...
            coefficientsLong0(3,5))-coefficientsLong0(3,4).*...
            (coefficientsLong0(3,1).*(SR+coefficientsLong0(3,5))...
            -atand(coefficientsLong0(3,1).*(SR+coefficientsLong0...
            (3,5))))))+coefficientsLong0(3,6);
        
        % IA = 2 Pure longitudinal matrix
        NFX250p = @(SR) coefficientsLong2(1,3).*sind(coefficientsLong2...
            (1,2).*atand(coefficientsLong2(1,1).*(SR+...
            coefficientsLong2(1,5))-coefficientsLong2(1,4).*...
            (coefficientsLong2(1,1).*(SR+coefficientsLong2(1,5))...
            -atand(coefficientsLong2(1,1).*(SR+coefficientsLong2...
            (1,5))))))+coefficientsLong2(1,6);
        
        NFX2150p = @(SR) coefficientsLong2(2,3).*sind(coefficientsLong2...
            (2,2).*atand(coefficientsLong2(2,1).*(SR+...
            coefficientsLong2(2,5))-coefficientsLong2(2,4).*...
            (coefficientsLong2(2,1).*(SR+coefficientsLong2(2,5))...
            -atand(coefficientsLong2(2,1).*(SR+coefficientsLong2...
            (2,5))))))+coefficientsLong2(2,6);
        
        NFX2250p = @(SR) coefficientsLong2(3,3).*sind(coefficientsLong2...
            (3,2).*atand(coefficientsLong2(3,1).*(SR+...
            coefficientsLong2(3,5))-coefficientsLong2(3,4).*...
            (coefficientsLong2(3,1).*(SR+coefficientsLong2(3,5))...
            -atand(coefficientsLong2(3,1).*(SR+coefficientsLong2...
            (3,5))))))+coefficientsLong2(3,6);
        
        % IA = 4 Pure longitudinal matrix
        NFX450p = @(SR) coefficientsLong4(1,3).*sind(coefficientsLong4...
            (1,2).*atand(coefficientsLong4(1,1).*(SR+...
            coefficientsLong4(1,5))-coefficientsLong4(1,4).*...
            (coefficientsLong4(1,1).*(SR+coefficientsLong4(1,5))...
            -atand(coefficientsLong4(1,1).*(SR+coefficientsLong4...
            (1,5))))))+coefficientsLong4(1,6);
        
        NFX4150p = @(SR) coefficientsLong4(2,3).*sind(coefficientsLong4...
            (2,2).*atand(coefficientsLong4(2,1).*(SR+...
            coefficientsLong4(2,5))-coefficientsLong4(2,4).*...
            (coefficientsLong4(2,1).*(SR+coefficientsLong4(2,5))...
            -atand(coefficientsLong4(2,1).*(SR+coefficientsLong4...
            (2,5))))))+coefficientsLong4(2,6);
        
        NFX4250p = @(SR) coefficientsLong4(3,3).*sind(coefficientsLong4...
            (3,2).*atand(coefficientsLong4(3,1).*(SR+...
            coefficientsLong4(3,5))-coefficientsLong4(3,4).*...
            (coefficientsLong4(3,1).*(SR+coefficientsLong4(3,5))...
            -atand(coefficientsLong4(3,1).*(SR+coefficientsLong4...
            (3,5))))))+coefficientsLong4(3,6);
        
        
        %% NFX Interpolation
        % IA between 0 and 2 deg and FZ between 50 and 150 lbs
        if (IA >= 0 && IA < 2) && (FZ <= -50 && FZ >= -150)
            NFX1 = interp1([0,2], [NFX050p(SR), NFX250p(SR)], IA);
            NFX2 = interp1([0,2], [NFX0150p(SR), NFX2150p(SR)], IA);
            NFX = interp1([-50, -150], [NFX1, NFX2], FZ);
            % IA between 2 and 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -50 && FZ >= -150)
            NFX1 = interp1([2,4], [NFX250p(SR), NFX450p(SR)], IA);
            NFX2 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA);
            NFX = interp1([-50, -150], [NFX1, NFX2], FZ);
            % IA above 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 4) && (FZ <= -50 && FZ >= -150)
            NFX1 = interp1([2,4], [NFX250p(SR), NFX450p(SR)], IA, 'linear', 'extrap');
            NFX2 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA, 'linear', 'extrap');
            NFX = interp1([-50, -150], [NFX1, NFX2], FZ);
            
            % IA between 0 and 2 deg and FZ between 150 and 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -150 && FZ >= -250)
            NFX1 = interp1([0,2], [NFX0150p(SR), NFX2150p(SR)], IA);
            NFX2 = interp1([0,2], [NFX0250p(SR), NFX2250p(SR)], IA);
            NFX = interp1([-150, -250], [NFX1, NFX2], FZ);
            % IA between 2 and 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -150 && FZ >= -250)
            NFX1 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA);
            NFX2 = interp1([2,4], [NFX2250p(SR), NFX4250p(SR)], IA);
            NFX = interp1([-150, -250], [NFX1, NFX2], FZ);
            % IA above 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 4) && (FZ <= -150 && FZ >= -250)
            NFX1 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA, 'linear', 'extrap');
            NFX2 = interp1([2,4], [NFX2250p(SR), NFX4250p(SR)], IA, 'linear', 'extrap');
            NFX = interp1([-150, -250], [NFX1, NFX2], FZ);
            
            % IA between 0 and 2 deg and FZ higher than 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -250)
            NFX = interp1([0,2], [NFX0Hp(SR), NFX2Hp(SR)], IA);
            
            % IA between 2 and 4 deg and FZ above 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -250)
            NFX = interp1([2,4], [NFX2Hp(SR), NFX4Hp(SR)], IA);
            
            % IA above 4 deg and FZ above 250 lbs
        elseif (IA >= 4) && (FZ <= -250)
            NFX = interp1([-250, FZ], [NFX4250p(SA), NFX4Hp(SA)], FZ);
            
            % IA between 0 and 2 deg and FZ below 50 lbs
        elseif (IA >= 0 && IA < 2) && (FZ >= -50)
            NFX1 = interp1([0,2], [NFX050p(SR), NFX250p(SR)], IA);
            NFX2 = interp1([0,2], [NFX0150p(SR), NFX2150p(SR)], IA);
            NFX = interp1([-50, -150], [NFX1, NFX2], FZ, 'linear', 'extrap');
            % IA between 2 and 4 deg and FZ below 50 lbs
        elseif (IA >= 2 && IA < 4) && (FZ >= -50)
            NFX1 = interp1([2,4], [NFX250p(SR), NFX450p(SR)], IA);
            NFX2 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA);
            NFX = interp1([-50, -150], [NFX1, NFX2], FZ, 'linear', 'extrap');
            % IA above 4 deg and FZ below 50 lbs
        elseif (IA >= 4) && (FZ > -50)
            NFX1 = interp1([2,4], [NFX250p(SR), NFX450p(SR)], IA, 'linear', 'extrap');
            NFX2 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA, 'linear', 'extrap');
            NFX = interp1([-50, -150], [NFX1, NFX2], FZ, 'linear', 'extrap');
        else
            disp(FZ)
            disp(IA)
            error('not all conditions accounted for')
        end
    end
    GY = 1;
    GX = 1;
    %% Combined Equations
    %% Combined Cornering
    
    % IA = 0 Combo Cornering Equations
    if SR ~= 0 && SA ~= 0
        GY050 = @(SR) cosd(ccc0(1,2).*atand(ccc0(1,1).*(SR+ccc0(1,4))-ccc0(1,3).*...
            (ccc0(1,1).*(SR+ccc0(1,4))-atand(ccc0(1,1).*(SR+ccc0(1,4))))))./...
            cosd(ccc0(1,2).*atand(ccc0(1,1).*ccc0(1,4) - ccc0(1,3).*(ccc0(1,1)...
            .*ccc0(1,4)-atand(ccc0(1,1).*ccc0(1,4)))));
        
        GY0150 = @(SR) cosd(ccc0(2,2).*atand(ccc0(2,1).*(SR+ccc0(2,4))-ccc0(2,3).*...
            (ccc0(2,1).*(SR+ccc0(2,4))-atand(ccc0(2,1).*(SR+ccc0(2,4))))))./...
            cosd(ccc0(2,2).*atand(ccc0(2,1).*ccc0(2,4) - ccc0(2,3).*(ccc0(2,1)...
            .*ccc0(2,4)-atand(ccc0(2,1).*ccc0(2,4)))));
        
        GY0250 = @(SR) cosd(ccc0(3,2).*atand(ccc0(3,1).*(SR+ccc0(3,4))-ccc0(3,3).*...
            (ccc0(3,1).*(SR+ccc0(3,4))-atand(ccc0(3,1).*(SR+ccc0(3,4))))))./...
            cosd(ccc0(3,2).*atand(ccc0(3,1).*ccc0(3,4) - ccc0(3,3).*(ccc0(3,1)...
            .*ccc0(3,4)-atand(ccc0(3,1).*ccc0(3,4)))));
        
        % IA = 2 Combo Cornering Equations
        GY250 = @(SR) cosd(ccc2(1,2).*atand(ccc2(1,1).*(SR+ccc2(1,4))-ccc2(1,3).*...
            (ccc2(1,1).*(SR+ccc2(1,4))-atand(ccc2(1,1).*(SR+ccc2(1,4))))))./...
            cosd(ccc2(1,2).*atand(ccc2(1,1).*ccc2(1,4) - ccc2(1,3).*(ccc2(1,1)...
            .*ccc2(1,4)-atand(ccc2(1,1).*ccc2(1,4)))));
        
        GY2150 = @(SR) cosd(ccc2(2,2).*atand(ccc2(2,1).*(SR+ccc2(2,4))-ccc2(2,3).*...
            (ccc2(2,1).*(SR+ccc2(2,4))-atand(ccc2(2,1).*(SR+ccc2(2,4))))))./...
            cosd(ccc2(2,2).*atand(ccc2(2,1).*ccc2(2,4) - ccc2(2,3).*(ccc2(2,1)...
            .*ccc2(2,4)-atand(ccc2(2,1).*ccc2(2,4)))));
        
        GY2250 = @(SR) cosd(ccc2(3,2).*atand(ccc2(3,1).*(SR+ccc2(3,4))-ccc2(3,3).*...
            (ccc2(3,1).*(SR+ccc2(3,4))-atand(ccc2(3,1).*(SR+ccc2(3,4))))))./...
            cosd(ccc2(3,2).*atand(ccc2(3,1).*ccc2(3,4) - ccc2(3,3).*(ccc2(3,1)...
            .*ccc2(3,4)-atand(ccc2(3,1).*ccc2(3,4)))));
        
        % IA = 4 Combo Cornering Equations
        GY450 = @(SR) cosd(ccc4(1,2).*atand(ccc4(1,1).*(SR+ccc4(1,4))-ccc4(1,3).*...
            (ccc4(1,1).*(SR+ccc4(1,4))-atand(ccc4(1,1).*(SR+ccc4(1,4))))))./...
            cosd(ccc4(1,2).*atand(ccc4(1,1).*ccc4(1,4) - ccc4(1,3).*(ccc4(1,1)...
            .*ccc4(1,4)-atand(ccc4(1,1).*ccc4(1,4)))));
        
        GY4150 = @(SR) cosd(ccc4(2,2).*atand(ccc4(2,1).*(SR+ccc4(2,4))-ccc4(2,3).*...
            (ccc4(2,1).*(SR+ccc4(2,4))-atand(ccc4(2,1).*(SR+ccc4(2,4))))))./...
            cosd(ccc4(2,2).*atand(ccc4(2,1).*ccc4(2,4) - ccc4(2,3).*(ccc4(2,1)...
            .*ccc4(2,4)-atand(ccc4(2,1).*ccc4(2,4)))));
        
        GY4250 = @(SR) cosd(ccc4(3,2).*atand(ccc4(3,1).*(SR+ccc4(3,4))-ccc4(3,3).*...
            (ccc4(3,1).*(SR+ccc4(3,4))-atand(ccc4(3,1).*(SR+ccc4(3,4))))))./...
            cosd(ccc4(3,2).*atand(ccc4(3,1).*ccc4(3,4) - ccc4(3,3).*(ccc4(3,1)...
            .*ccc4(3,4)-atand(ccc4(3,1).*ccc4(3,4)))));
        
        %% GY Interpolation
        % IA between 0 and 2 deg and FZ between 50 and 150 lbs
        if (IA >= 0 && IA < 2) && (FZ <= -50 && FZ >= -150)
            GY = interp1([0,2], [GY050(SR), GY250(SR)], IA);
            GY2 = interp1([0,2], [GY0150(SR), GY2150(SR)], IA);
            GY = interp1([-50, -150], [GY, GY2], FZ);
            % IA between 2 and 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -50 && FZ >= -150)
            GY = interp1([2,4], [GY250(SR), GY450(SR)], IA);
            GY2 = interp1([2,4], [GY2150(SR), GY4150(SR)], IA);
            GY = interp1([-50, -150], [GY, GY2], FZ);
            % IA above 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 4) && (FZ <= -50 && FZ >= -150)
            GY = interp1([2,4], [GY250(SR), GY450(SR)], IA, 'linear', 'extrap');
            GY2 = interp1([2,4], [GY2150(SR), GY4150(SR)], IA, 'linear', 'extrap');
            GY = interp1([-50, -150], [GY, GY2], FZ);
            
            % IA between 0 and 2 deg and FZ between 150 and 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -150 && FZ >= -250)
            GY = interp1([0,2], [GY0150(SR), GY2150(SR)], IA);
            GY2 = interp1([0,2], [GY0250(SR), GY2250(SR)], IA);
            GY = interp1([-150, -250], [GY, GY2], FZ);
            % IA between 2 and 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -150 && FZ >= -250)
            GY = interp1([2,4], [GY2150(SR), GY4150(SR)], IA);
            GY2 = interp1([2,4], [GY2250(SR), GY4250(SR)], IA);
            GY = interp1([-150, -250], [GY, GY2], FZ);
            % IA above 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 4) && (FZ <= -150 && FZ >= -250)
            GY = interp1([2,4], [GY2150(SR), GY4150(SR)], IA, 'linear', 'extrap');
            GY2 = interp1([2,4], [GY2250(SR), GY4250(SR)], IA, 'linear', 'extrap');
            GY = interp1([-150, -250], [GY, GY2], FZ);
            
            % IA between 0 and 2 deg and FZ above 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -250)
            GY = interp1([0,2], [GY0150(SR), GY2150(SR)], IA);
            GY2 = interp1([0,2], [GY0250(SR), GY2250(SR)], IA);
            GY = interp1([-150, -250], [GY, GY2], FZ, 'linear', 'extrap');
            % IA between 2 and 4 deg and FZ above 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -250)
            GY = interp1([2,4], [GY2150(SR), GY4150(SR)], IA);
            GY2 = interp1([2,4], [GY2250(SR), GY4250(SR)], IA);
            GY = interp1([-150, -250], [GY, GY2], FZ, 'linear', 'extrap');
            % IA above 4 deg and FZ above 250 lbs
        elseif (IA >= 4) && (FZ <= -250)
            GY = interp1([2,4], [GY2150(SR), GY4150(SR)], IA, 'linear', 'extrap');
            GY2 = interp1([2,4], [GY2250(SR), GY4250(SR)], IA, 'linear', 'extrap');
            GY = interp1([-150, -250], [GY, GY2], FZ, 'linear', 'extrap');
            
            % IA between 0 and 2 deg and FZ below 50 lbs
        elseif (IA >= 0 && IA < 2) && (FZ >= -50)
            GY = interp1([0,2], [GY050(SR), GY250(SR)], IA);
            GY2 = interp1([0,2], [GY0150(SR), GY2150(SR)], IA);
            GY = interp1([-50, -150], [GY, GY2], FZ, 'linear', 'extrap');
            % IA between 2 and 4 deg and FZ below 50 lbs
        elseif (IA >= 2 && IA < 4) && (FZ >= -50)
            GY = interp1([2,4], [GY250(SR), GY450(SR)], IA);
            GY2 = interp1([2,4], [GY2150(SR), GY4150(SR)], IA);
            GY = interp1([-50, -150], [GY, GY2], FZ, 'linear', 'extrap');
            % IA above 4 deg and FZ below 50 lbs
        elseif (IA >= 4) && (FZ > -50)
            GY = interp1([2,4], [GY250(SR), GY450(SR)], IA, 'linear', 'extrap');
            GY2 = interp1([2,4], [GY2150(SR), GY4150(SR)], IA, 'linear', 'extrap');
            GY = interp1([-50, -150], [GY, GY2], FZ, 'linear', 'extrap');
        else
            disp(FZ)
            disp(IA)
            error('not all conditions accounted for')
        end
        
        %% Combined Longitudinal
        
        % IA = 0 Combo Longitudinal Equations
        GX050 = @(SA) cosd(ccl0(1,2).*atand(ccl0(1,1).*(SA+ccl0(1,4))-ccl0(1,3).*...
            (ccl0(1,1).*(SA+ccl0(1,4))-atand(ccl0(1,1).*(SA+ccl0(1,4))))))./...
            cosd(ccl0(1,2).*atand(ccl0(1,1).*ccl0(1,4) - ccl0(1,3).*(ccl0(1,1)...
            .*ccl0(1,4)-atand(ccl0(1,1).*ccl0(1,4)))));
        
        GX0150 = @(SA) cosd(ccl0(2,2).*atand(ccl0(2,1).*(SA+ccl0(2,4))-ccl0(2,3).*...
            (ccl0(2,1).*(SA+ccl0(2,4))-atand(ccl0(2,1).*(SA+ccl0(2,4))))))./...
            cosd(ccl0(2,2).*atand(ccl0(2,1).*ccl0(2,4) - ccl0(2,3).*(ccl0(2,1)...
            .*ccl0(2,4)-atand(ccl0(2,1).*ccl0(2,4)))));
        
        GX0250 = @(SA) cosd(ccl0(3,2).*atand(ccl0(3,1).*(SA+ccl0(3,4))-ccl0(3,3).*...
            (ccl0(3,1).*(SA+ccl0(3,4))-atand(ccl0(3,1).*(SA+ccl0(3,4))))))./...
            cosd(ccl0(3,2).*atand(ccl0(3,1).*ccl0(3,4) - ccl0(3,3).*(ccl0(3,1)...
            .*ccl0(3,4)-atand(ccl0(3,1).*ccl0(3,4)))));
        
        % IA = 2 Combo Longitudinal Equations
        GX250 = @(SA) cosd(ccl2(1,2).*atand(ccl2(1,1).*(SA+ccl2(1,4))-ccl2(1,3).*...
            (ccl2(1,1).*(SA+ccl2(1,4))-atand(ccl2(1,1).*(SA+ccl2(1,4))))))./...
            cosd(ccl2(1,2).*atand(ccl2(1,1).*ccl2(1,4) - ccl2(1,3).*(ccl2(1,1)...
            .*ccl2(1,4)-atand(ccl2(1,1).*ccl2(1,4)))));
        
        GX2150 = @(SA) cosd(ccl2(2,2).*atand(ccl2(2,1).*(SA+ccl2(2,4))-ccl2(2,3).*...
            (ccl2(2,1).*(SA+ccl2(2,4))-atand(ccl2(2,1).*(SA+ccl2(2,4))))))./...
            cosd(ccl2(2,2).*atand(ccl2(2,1).*ccl2(2,4) - ccl2(2,3).*(ccl2(2,1)...
            .*ccl2(2,4)-atand(ccl2(2,1).*ccl2(2,4)))));
        
        GX2250 = @(SA) cosd(ccl2(3,2).*atand(ccl2(3,1).*(SA+ccl2(3,4))-ccl2(3,3).*...
            (ccl2(3,1).*(SA+ccl2(3,4))-atand(ccl2(3,1).*(SA+ccl2(3,4))))))./...
            cosd(ccl2(3,2).*atand(ccl2(3,1).*ccl2(3,4) - ccl2(3,3).*(ccl2(3,1)...
            .*ccl2(3,4)-atand(ccl2(3,1).*ccl2(3,4)))));
        
        % IA = 4 Combo Longitudinal Equations
        GX450 = @(SA) cosd(ccl4(1,2).*atand(ccl4(1,1).*(SA+ccl4(1,4))-ccl4(1,3).*...
            (ccl4(1,1).*(SA+ccl4(1,4))-atand(ccl4(1,1).*(SA+ccl4(1,4))))))./...
            cosd(ccl4(1,2).*atand(ccl4(1,1).*ccl4(1,4) - ccl4(1,3).*(ccl4(1,1)...
            .*ccl4(1,4)-atand(ccl4(1,1).*ccl4(1,4)))));
        
        GX4150 = @(SA) cosd(ccl4(2,2).*atand(ccl4(2,1).*(SA+ccl4(2,4))-ccl4(2,3).*...
            (ccl4(2,1).*(SA+ccl4(2,4))-atand(ccl4(2,1).*(SA+ccl4(2,4))))))./...
            cosd(ccl4(2,2).*atand(ccl4(2,1).*ccl4(2,4) - ccl4(2,3).*(ccl4(2,1)...
            .*ccl4(2,4)-atand(ccl4(2,1).*ccl4(2,4)))));
        
        GX4250 = @(SA) cosd(ccl4(3,2).*atand(ccl4(3,1).*(SA+ccl4(3,4))-ccl4(3,3).*...
            (ccl4(3,1).*(SA+ccl4(3,4))-atand(ccl4(3,1).*(SA+ccl4(3,4))))))./...
            cosd(ccl4(3,2).*atand(ccl4(3,1).*ccl4(3,4) - ccl4(3,3).*(ccl4(3,1)...
            .*ccl4(3,4)-atand(ccl4(3,1).*ccl4(3,4)))));
        
        %% GX Interpolation
        % IA between 0 and 2 deg and FZ between 50 and 150 lbs
        if (IA >= 0 && IA < 2) && (FZ <= -50 && FZ >= -150)
            GX = interp1([0,2], [GX050(SA), GX250(SA)], IA);
            GX2 = interp1([0,2], [GX0150(SA), GX2150(SA)], IA);
            GX = interp1([-50, -150], [GX, GX2], FZ);
            % IA between 2 and 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -50 && FZ >= -150)
            GX = interp1([2,4], [GX250(SA), GX450(SA)], IA);
            GX2 = interp1([2,4], [GX2150(SA), GX4150(SA)], IA);
            GX = interp1([-50, -150], [GX, GX2], FZ);
            % IA above 4 deg and FZ between 50 and 150 lbs
        elseif (IA >= 4) && (FZ <= -50 && FZ >= -150)
            GX = interp1([-50,-150], [GX450(SA), GX4150(SA)], FZ);
            
            % IA between 0 and 2 deg and FZ between 150 and 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -150 && FZ >= -250)
            GX = interp1([0,2], [GX0150(SA), GX2150(SA)], IA);
            GX2 = interp1([0,2], [GX0250(SA), GX2250(SA)], IA);
            GX = interp1([-150, -250], [GX, GX2], FZ);
            % IA between 2 and 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -150 && FZ >= -250)
            GX = interp1([2,4], [GX2150(SA), GX4150(SA)], IA);
            GX2 = interp1([2,4], [GX2250(SA), GX4250(SA)], IA);
            GX = interp1([-150, -250], [GX, GX2], FZ);
            % IA above 4 deg and FZ between 150 and 250 lbs
        elseif (IA >= 4) && (FZ <= -150 && FZ >= -250)
            GX = interp1([-150,-250], [GX4150(SA), GX4250(SA)], FZ);
            
            % IA between 0 and 2 deg and FZ above 250 lbs
        elseif (IA >= 0 && IA < 2) && (FZ <= -250)
            GX = interp1([0,2], [GX0150(SA), GX2150(SA)], IA);
            GX2 = interp1([0,2], [GX0250(SA), GX2250(SA)], IA);
            GX = interp1([-150, -250], [GX, GX2], FZ, 'linear', 'extrap');
            % IA between 2 and 4 deg and FZ above 250 lbs
        elseif (IA >= 2 && IA < 4) && (FZ <= -250)
            GX = interp1([2,4], [GX2150(SA), GX4150(SA)], IA);
            GX2 = interp1([2,4], [GX2250(SA), GX4250(SA)], IA);
            GX = interp1([-150, -250], [GX, GX2], FZ, 'linear', 'extrap');
            % IA above 4 deg and FZ above 250 lbs
        elseif (IA >= 4) && (FZ <= -250)
            GX = interp1([-150,-250], [GX4150(SA), GX4250(SA)], FZ, 'linear', 'extrap');
            
            % IA between 0 and 2 deg and FZ below 50 lbs
        elseif (IA >= 0 && IA < 2) && (FZ >= -50)
            GX = interp1([0,2], [GX050(SA), GX250(SA)], IA);
            GX2 = interp1([0,2], [GX0150(SA), GX2150(SA)], IA);
            GX = interp1([-50, -150], [GX, GX2], FZ, 'linear', 'extrap');
            % IA between 2 and 4 deg and FZ below 50 lbs
        elseif (IA >= 2 && IA < 4) && (FZ >= -50)
            GX = interp1([2,4], [GX250(SA), GX450(SA)], IA);
            GX2 = interp1([2,4], [GX2150(SA), GX4150(SA)], IA);
            GX = interp1([-50, -150], [GX, GX2], FZ, 'linear', 'extrap');
            % IA above 4 deg and FZ below 50 lbs
        elseif (IA >= 4) && (FZ > -50)
            GX = interp1([-150,-50], [GX4150(SA), GX450(SA)], FZ);
        else
            disp(FZ)
            disp(IA)
            error('not all conditions accounted for')
        end
    end
    if SA == 0
        NFY = 0;
    end
    if SR == 0
        NFX = 0;
    end
    
    NFY(index) = NFY*GY*(1/5); % The 1/5 factor is the correction factor for asphalt ~ish
    NFX(index) = NFX*GX*(1/5);
end

NFY = mean(abs(NFY));
NFX = mean(abs(NFX));
if correction == 1
    NFX = -NFX;
    NFY = -NFY;
end

if FZcorrect == 1
    NFY = NFY*(-3E-11*(FZo^3) + 2E-07*(FZo^2) - 0.0007*FZo + 1.1671);
    NFX = NFX*(-3E-11*(FZo^3) + 2E-07*(FZo^2) - 0.0007*FZo + 1.1671);
    
    % An explanation: This code is an extrapolation of the data, there is
    % about a factor of 0.95 per 100lbs. This math uses FZ with a fitted
    % curve in excel to adjust NFY to a more "realistic" number. This does
    % need to be VALIDATED MOTHAFUCKAAAAAAAAA
end

PT = (0.0022*(abs(SA)^3)-.0362*(abs(SA)^2)-.0078*(abs(SA))+1.8239);
if SA>=0
    MZ = abs(PT*(NFY*FZ));
else
    MZ = -abs(PT*(NFY*FZ));
end

FX = NFX*FZ;
FY = NFY*FZ;

end


