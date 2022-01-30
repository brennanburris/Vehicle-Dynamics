function [coefficientsCWF] = tireModel2()
%Brennan Burris
%1/22/2022
%This function solves for combined slip of a single tire based on TTC data.
%Utilizing a numerical optimizer, this function solves for the parameters
%of the pure slip data Pacejka Magic Formula as well as the combined slip
%equation parameters. 

%load in tire data. Brake data is preffered as it includes different slip
%angles as well as slip ratios, making it a good candidate for a complete
%pure slip model

clear
close all
clc

load('B1654run9')
MFCsolver()
parametersSA = ans;
SAy = SA;
SRy = SR;
ETy = ET;
NFYy = NFY;
clearvars -except parametersSA SAy SRy NFYy ETy

load('B1654run56')
MFAsolver()
parametersSR = ans;
SAx = SA;
SRx = SR;
ETx = ET;
clearvars -except parametersSR SAx SRx parametersSA SAy SRy NFYy...
    NFX ETy ETx
NFY = NFYy;
clearvars NFYy

%Start initial guesses for cosine weighing function parameters
Bxa = 8;
Cxa = .5;
Exa = .5;
Shxa = 0;
Svxa = 0;

Byk = 8;
Cyk = .5;
Eyk = -.5;
Shyk = 0;
Svyk = 0;

%End cosine weighing function parameters

%initialize parameters
x0x = [Bxa, Cxa, Exa, Shxa, Svxa];
x0y = [Byk, Cyk, Eyk, Shyk, Svyk];

xx = x0x;
xy = x0y;

%x combined slip
muxFunc = @(xx) (parametersSR(3).*sind(parametersSR(2).*atand(parametersSR...
    (1).*(SRx+parametersSR(5)-parametersSR(4).*(parametersSR(1).*(SRx+...
    parametersSR(5)-atand(parametersSR(1).*(SRx+parametersSR(5))))))+...
    parametersSR(6)))) .* ...
    ((cosd(xx(2).*atand(xx(1).*(SAx+xx(5))-xx(4).*(xx(1).*(SAx+...
    xx(5))-atand(xx(1).*(SAx+xx(5)))))))./(cosd(xx(2).*atand(xx(1)...
    .*xx(5)-xx(4).*(xx(1).*xx(5)-atand(xx(1).*xx(5)))))));


%y combined slip
muyFunc = @(xy) (parametersSA(3).*sind(parametersSA(2).*atand(parametersSA...
    (1).*(SAy+parametersSA(5)-parametersSA(4).*(parametersSA(1).*(SAy+...
    parametersSA(5)-atand(parametersSA(1).*(SAy+parametersSA(5))))))+...
    parametersSA(6)))) .* ...
    ((cosd(xy(2).*atand(xy(1).*(SRy+xy(5))-xy(4).*(xy(1).*(SRy+...
    xy(5))-atand(xy(1).*(SRy+xy(5)))))))./(cosd(xy(2).*atand(xy(1)...
    .*xy(5)-xy(4).*(xy(1).*xy(5)-atand(xy(1).*xy(5)))))));

errorx = @(x0x) sqrt(sum((muxFunc(x0x) - NFX).^2));
errory = @(x0y) sqrt(sum((muyFunc(x0y) - NFY).^2));
 
gs = GlobalSearch;
problem1 = createOptimProblem('fmincon', 'x0', x0x,'objective', errorx,...
    'lb', [], 'ub', []);
problem2 = createOptimProblem('fmincon', 'x0', x0y,'objective', errory,...
    'lb', [], 'ub', []);
coefficientsCWF = ones(size(xx));
xx = run(gs, problem1);
xy = run(gs, problem2);
coefficientsCWF = xx.*(coefficientsCWF(1,:));
coefficientsCWF = xy.*(coefficientsCWF(2,:));
figure
subplot(2,1,1)
plot(ETx, NFX);
hold on
plot(ETx, muxFunc(xx));
title('Time vs FX')
subplot(2,1,2);
plot(ETy, NFY);
hold on
plot(ETy, muyFunc(xy));
title('Time vs FY')
hold off
end

