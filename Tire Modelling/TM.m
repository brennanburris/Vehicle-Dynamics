function [NFX, NFY] = TM(FZ, SA, SR, IA)
%% THE Tire Model
%% Brennan Burris
% Ram Racing, 9/11/2022
% brennanburris@gmail.com

%% Definitions
% SA = Slip angle
% SR = Slip ratio
% FZ = normal force
% IA = camber angle 
%% Description
% This is the result of a lot of hard work. It includes pure slip equations
% for longitudinal and lateral acceleration, as well as the combined slip
% equations.

% Each equation represents tires at different inclination angles and
% different normal forces. Based on the inputs of FZ and IA, this model
% will output the force the tire can create given these parameters. 

% Inputs are FZ, SA, and SR
% The first number after NFY denotes the inclination angle. The first
% letter, either p or c denotes pure slip (p) or combined slip (c).
% Therefore, NFY0p would stand for NFY at IA = 0 in pure slip under
% cornering. NFX4c would stand for NFX at IA = 4 under combined slip. 

%% NFY = D.*sind(C.*atand(B.*(SA+...
%  Sh)-E.*(B.*(SA+sh)...
%  -atand(B.*(SA+Sh)))))+Sv

%% Housekeeping
if nargin ~= 4
    error('Not enough input arguments')
end
FZ = -1*abs(FZ);
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
%% 

coefficientsCornering0 = [0.124871400248252,1.50000048303276,2.52217149344829,0.0131321089998474,0.0273183419110165,0.00353815752754545;0.108342744408079,1.50000004291032,2.43273335153386,0.00939970739262675,-0.0423876339249668,0.0291441536132132;0.00127787666246645,1.83127180640110,2.36920063404591,1.31324995878059,0.0182258971991196,-0.0174717352168762];
coefficientsCornering2 = [0.119333911373298,1.50000002832375,2.41325957351056,0.0122501549335187,0.0417914321576790,-0.0351444274142365;0.100637701307224,1.60000015502581,2.36789891734450,0.00979987631457392,-0.213120656298526,0.0346853817476196;0.0708463663938786,2.49997849218144,2.30027088430659,2.64782017943538e-07,-0.390506351212477,0.0337235212066137];
coefficientsCornering4 = [0.119582739537369,1.45000018076451,2.30330464551135,0.0111695362812682,-0.0345703122444912,-0.0222785208382582;0.00178965252122583,1.49981859188847,2.21951129591912,1.66587891745016,-0.215814026503531,-0.0473363998754018;0.00564366644391225,2.08395746853433,2.25168281790795,0.252788418300819,-0.658376733488556,0.0416464510859657];
coefficientsLong0 = [8.49950959059766,1.75000032410059,-2.03579172370720,0.0138948889989628,-0.0235574692592698,0.0687226771774872;8.49960000000000,1.57800000000000,-2.30000000000000,0.0156000000000000,-0.0114000000000000,0.0618000000000000;4.61261975097272,1.70106061786626,-2.29188461972766,0.0354761676530278,0.000547345394233733,0.111517994952401];
coefficientsLong2 = [8.49999230950463,1.50000011465249,-1.98468235181166,0.0247640721207096,-0.0264373731144383,0.0773635209066024;8.49926901869931,1.94736190886371,-2.36024392333249,0.00690956126379126,-0.0143791353267913,0.0510672353809388;8.22147767172531,1.75001394244138,-2.33318518258399,0.0127707325439920,-0.00653950249428973,0.0400147937006177];
coefficientsLong4 = [8.05987216361744,1.75007806742004,-1.98866565572401,0.0136204759577897,-0.0344081978158173,0.0630288517275184;7.49297502182196,1.75000201295265,-2.27923555906740,0.0165838853317732,-0.0116622399432688,0.0538548388445395;6.96595101484201,1.74992596179042,-2.61226150246255,0.0201920442097409,-0.00395277431333100,0.0460161273365707];
ccc0 = [1.01522681545190,1.40766930476357,0.249657608214171,0.0472634056656552,0.0500000000000000;42.2603420252637,12.9215250701272,67.3303393841706,-2.12060453806994,0.0500000000000000;-0.501655372725928,5.82093689196927,-0.240050993913462,-0.0644980569124586,0.0500000000000000];
ccc2 = [5.48088380641448,1.04840611075299,0.0156952514909401,0.0339816789928007,0.0500000000000000;4.61847089280628,1.13652767125759,0.0145244867962192,0.0328134865032152,0.0500000000000000;1.66547655602971,1.53459808601567,0.0663795332851410,0.0557409180810074,0.0500000000000000];
ccc4 = [-2.80544203664387,-4998.99999786752,6211.29934674600,9458.06017985634,2522.35372073046;3.09790219548835,0.981994881885186,0.0338296008062256,0.00762296760851838,0.0500000000000000;0.961310984668721,1.15150495869317,0.267064852030244,0.0379587998792334,0.0500000000000000];
ccl0 = [1.01522681545190,1.40766930476357,0.249657608214171,0.0472634056656552,0.0500000000000000;42.2603420252637,12.9215250701272,67.3303393841706,-2.12060453806994,0.0500000000000000;-0.501655372725928,5.82093689196927,-0.240050993913462,-0.0644980569124586,0.0500000000000000];
ccl2 = [5.48088380641448,1.04840611075299,0.0156952514909401,0.0339816789928007,0.0500000000000000;4.61847089280628,1.13652767125759,0.0145244867962192,0.0328134865032152,0.0500000000000000;1.66547655602971,1.53459808601567,0.0663795332851410,0.0557409180810074,0.0500000000000000];
ccl4 = [-2.80544203664387,-4998.99999786752,6211.29934674600,9458.06017985634,2522.35372073046;3.09790219548835,0.981994881885186,0.0338296008062256,0.00762296760851838,0.0500000000000000;0.961310984668721,1.15150495869317,0.267064852030244,0.0379587998792334,0.0500000000000000];


IA = abs(IA);
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
        NFY1 = interp1([2,4], [NFY250p(SA), NFY450p(SA)], IA, 'linear', 'extrap');
        NFY2 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA, 'linear', 'extrap');
        NFY = interp1([-50, -150], [NFY1, NFY2], FZ);

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
        NFY1 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA, 'linear', 'extrap');
        NFY2 = interp1([2,4], [NFY2250p(SA), NFY4250p(SA)], IA, 'linear', 'extrap');
        NFY = interp1([-150, -250], [NFY1, NFY2], FZ);
        
% IA between 0 and 2 deg and FZ above 250 lbs
    elseif (IA >= 0 && IA < 2) && (FZ <= -250)
        NFY1 = interp1([0,2], [NFY0150p(SA), NFY2150p(SA)], IA);
        NFY2 = interp1([0,2], [NFY0250p(SA), NFY2250p(SA)], IA);
        NFY = interp1([-150, -250], [NFY1, NFY2], FZ, 'linear', 'extrap');
% IA between 2 and 4 deg and FZ above 250 lbs 
    elseif (IA >= 2 && IA < 4) && (FZ <= -250)
        NFY1 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA);
        NFY2 = interp1([2,4], [NFY2250p(SA), NFY4250p(SA)], IA);
        NFY = interp1([-150, -250], [NFY1, NFY2], FZ, 'linear', 'extrap');
% IA above 4 deg and FZ above 250 lbs        
    elseif (IA >= 4) && (FZ <= -250)
        NFY1 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA, 'linear', 'extrap');
        NFY2 = interp1([2,4], [NFY2250p(SA), NFY4250p(SA)], IA, 'linear', 'extrap');
        NFY = interp1([-150, -250], [NFY1, NFY2], FZ, 'linear', 'extrap');

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
        NFY1 = interp1([2,4], [NFY250p(SA), NFY450p(SA)], IA, 'linear', 'extrap');
        NFY2 = interp1([2,4], [NFY2150p(SA), NFY4150p(SA)], IA, 'linear', 'extrap');
        NFY = interp1([-50, -150], [NFY1, NFY2], FZ, 'linear', 'extrap');
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
        
% IA between 0 and 2 deg and FZ above 250 lbs
    elseif (IA >= 0 && IA < 2) && (FZ <= -250)
        NFX1 = interp1([0,2], [NFX0150p(SR), NFX2150p(SR)], IA);
        NFX2 = interp1([0,2], [NFX0250p(SR), NFX2250p(SR)], IA);
        NFX = interp1([-150, -250], [NFX1, NFX2], FZ, 'linear', 'extrap');
% IA between 2 and 4 deg and FZ above 250 lbs 
    elseif (IA >= 2 && IA < 4) && (FZ <= -250)
        NFX1 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA);
        NFX2 = interp1([2,4], [NFX2250p(SR), NFX4250p(SR)], IA);
        NFX = interp1([-150, -250], [NFX1, NFX2], FZ, 'linear', 'extrap');
% IA above 4 deg and FZ above 250 lbs        
    elseif (IA >= 4) && (FZ <= -250)
        NFX1 = interp1([2,4], [NFX2150p(SR), NFX4150p(SR)], IA, 'linear', 'extrap');
        NFX2 = interp1([2,4], [NFX2250p(SR), NFX4250p(SR)], IA, 'linear', 'extrap');
        NFX = interp1([-150, -250], [NFX1, NFX2], FZ, 'linear', 'extrap');

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
        GX = interp1([2,4], [GX250(SA), GX450(SA)], IA, 'linear', 'extrap');
        GX2 = interp1([2,4], [GX2150(SA), GX4150(SA)], IA, 'linear', 'extrap');
        GX = interp1([-50, -150], [GX, GX2], FZ);

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
        GX = interp1([2,4], [GX2150(SA), GX4150(SA)], IA, 'linear', 'extrap');
        GX2 = interp1([2,4], [GX2250(SA), GX4250(SA)], IA, 'linear', 'extrap');
        GX = interp1([-150, -250], [GX, GX2], FZ);
        
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
        GX = interp1([2,4], [GX2150(SA), GX4150(SA)], IA, 'linear', 'extrap');
        GX2 = interp1([2,4], [GX2250(SA), GX4250(SA)], IA, 'linear', 'extrap');
        GX = interp1([-150, -250], [GX, GX2], FZ, 'linear', 'extrap');

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
        GX = interp1([2,4], [GX250(SA), GX450(SA)], IA, 'linear', 'extrap');
        GX2 = interp1([2,4], [GX2150(SA), GX4150(SA)], IA, 'linear', 'extrap');
        GX = interp1([-50, -150], [GX, GX2], FZ, 'linear', 'extrap');
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

NFY = NFY*GY/1.92;
NFX = NFX*GX/1.92;
end


