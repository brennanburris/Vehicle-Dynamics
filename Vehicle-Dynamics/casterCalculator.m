function [MZ0,MZ3,MZ42,MZ5,MZ6,MZ7,MZ8] = casterCalculator(pinionD, wheelD, swD)
%% Caster + MZ Calculator 

% Brennan Burris
% brennanburris@gmail.com

% This script was built in conjuntion with the tire model to give engineers
% an idea of how the self aligning moment will change based on the caster
% angle of the tire. 

%% Definitions
% caster: should be an array with multiple caster angles in degrees
% FZ: the amount of weight you expect to see on the tire in lbs
% pinionD: the diameter of the rack pinion gear in inches
% wheelD: the diameter of the wheel + tire in inches
% swD: the diameter of the steering wheel in inches

clc
close all

SAarray = -10:.25:10;
SR = 0;
IA = 0;
FZL = 600/4; % Assumed weight over front of car 
FZR = FZL;
lever = 2.5; % Steering lever in upright

for i = 1:length(SAarray)
[NFXL(i), FX(i), NFYL(i), FYL(i), MZL(i), PTL(i)] = TM(FZL, SAarray(i), SR, IA);
MZ0L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(0)));
MZ3L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(3)));
MZ42L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(4.2)));
MZ5L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(5)));
MZ6L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(6)));
MZ7L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(7)));
MZ8L(i) = (-FZL*NFYL(i))*(PTL(i)+((wheelD/2)*tand(8)));

[NFXR(i), FX(i), NFYR(i), FYR(i), MZR(i), PTR(i)] = TM(FZR, SAarray(i), SR, IA);
MZ0R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(0)));
MZ3R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(3)));
MZ42R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(4.2)));
MZ5R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(5)));
MZ6R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(6)));
MZ7R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(7)));
MZ8R(i) = (-FZR*NFYR(i))*(PTR(i)+((wheelD/2)*tand(8)));

FY(i) = (FYL(i)+FYR(i));

MZ0(i) = MZ0L(i) + MZ0R(i);
MZ3(i) = MZ3L(i) + MZ3R(i);
MZ42(i) = MZ42L(i) + MZ42R(i);
MZ5(i) = MZ5L(i) + MZ5R(i);
MZ6(i) = MZ6L(i) + MZ6R(i);
MZ7(i) = MZ7L(i) + MZ7R(i);
MZ8(i) = MZ8L(i) + MZ8R(i);

end

[MZ3pks,MZ3locs] = findpeaks(MZ3);
[MZ42pks, MZ42locs] = findpeaks(MZ42);
[MZ5pks, MZ5locs] = findpeaks(MZ5);
[MZ6pks, MZ6locs] = findpeaks(MZ6);
[MZ7pks,MZ7locs] = findpeaks(MZ7);
[MZ8pks, MZ8locs] = findpeaks(MZ8);

plot(SAarray,MZ0 )
hold on
plot(SAarray, MZ3)
hold on
plot(SAarray,MZ42)
hold on
plot(SAarray,MZ5)
hold on
plot(SAarray, MZ6)
hold on
plot(SAarray,MZ7)
hold on
plot(SAarray,MZ8)
hold off

legend('0 deg Caster ','3 deg Caster', '4 deg Caster', '5 deg Caster','6 deg Caster', '7 deg Caster', '8 deg Caster')
title('Aligning Moment vs Slip Angle')
ylabel('Aligning Moment (in*lbs)')
xlabel('Slip Angle (deg)')


%% Steering Force Calculations

for i = 1:length(MZ6L)
    
    % 3 deg caster
FLL3(i) = -MZ3L(i)/lever; % Force thru left link
FRL3(i) = -MZ3R(i)/lever;

Mrack3(i) = (-FLL3(i)-FRL3(i))*(pinionD/2); % Steering Rack Moment

steerEffort3(i) = (Mrack3(i));

% 4 deg caster
FLL42(i) = -MZ42L(i)/lever; % Force thru left link
FRL42(i) = -MZ42R(i)/lever;

Mrack42(i) = (-FLL42(i)-FRL42(i))*(pinionD/2); % Steering Rack Moment

steerEffort42(i) = (Mrack42(i));

% 8 deg caster
FLL5(i) = -MZ5L(i)/lever; % Force thru left link
FRL5(i) = -MZ5R(i)/lever;

Mrack5(i) = (-FLL5(i)-FRL5(i))*(pinionD/2); % Steering Rack Moment

steerEffort5(i) = (Mrack5(i));
    
% 6 deg caster
FLL6(i) = -MZ6L(i)/lever; % Force thru left link
FRL6(i) = -MZ6R(i)/lever;

Mrack6(i) = (-FLL6(i)-FRL6(i))*(pinionD/2); % Steering Rack Moment

steerEffort6(i) = (Mrack6(i));

% 7 deg caster
FLL7(i) = -MZ7L(i)/lever; % Force thru left link
FRL7(i) = -MZ7R(i)/lever;

Mrack7(i) = (-FLL7(i)-FRL7(i))*(pinionD/2); % Steering Rack Moment

steerEffort7(i) = (Mrack7(i));

% 8 deg caster
FLL8(i) = -MZ8L(i)/lever; % Force thru left link
FRL8(i) = -MZ8R(i)/lever;

Mrack8(i) = (-FLL8(i)-FRL8(i))*(pinionD/2); % Steering Rack Moment

steerEffort8(i) = (Mrack8(i));


end

[steerEffortpks3, steerEffortlocs3] = findpeaks(steerEffort3);
[steerEffortpks42, steerEffortlocs42] = findpeaks(steerEffort42);
[steerEffortpks5, steerEffortlocs5] = findpeaks(steerEffort5);
[steerEffortpks6, steerEffortlocs6] = findpeaks(steerEffort6);
[steerEffortpks7, steerEffortlocs7] = findpeaks(steerEffort7);
[steerEffortpks8, steerEffortlocs8] = findpeaks(steerEffort8);


figure
plot(SAarray, steerEffort3)
hold on
plot(SAarray, steerEffort42)
hold on
plot(SAarray, steerEffort5)
hold on
plot(SAarray, steerEffort6)
hold on
plot(SAarray, steerEffort7)
hold on
plot(SAarray, steerEffort8)
hold on
scatter(SAarray(steerEffortlocs3),steerEffortpks3,200)
hold on
scatter(SAarray(steerEffortlocs42),steerEffortpks42,200)
hold on
scatter(SAarray(steerEffortlocs5),steerEffortpks5,200)
hold on
scatter(SAarray(steerEffortlocs6),steerEffortpks6,200)
hold on
scatter(SAarray(steerEffortlocs7),steerEffortpks7,200)
hold on
scatter(SAarray(steerEffortlocs8),steerEffortpks8,200)
hold off
title('Steering Effort vs Slip Angle')
ylabel('Steering Effort at Wheel (lbs*in)')
xlabel('Slip Angle (deg)')
legend('MZ 3 degrees of Caster', 'MZ 4.2 degrees of Caster', 'MZ 5 degrees of Caster','MZ 6 degrees of Caster', 'MZ 7 degrees of Caster', 'MZ 8 degrees of Caster')


% figure
% plot(SAarray, MZ3/max(MZ3))
% hold on
% plot(SAarray,MZ4/max(MZ4))
% hold on
% plot(SAarray,MZ5/max(MZ5))
% hold on
% plot(SAarray, MZ6/max(MZ6))
% hold on
% plot(SAarray,MZ7/max(MZ7))
% hold on
% plot(SAarray,MZ8/max(MZ8))
% hold on
% plot(SAarray, FY/max(FY))
% legend('MZ 3 degrees of Caster', 'MZ 4 degrees of Caster', 'MZ 5 degrees of Caster','MZ 6 degrees of Caster', 'MZ 7 degrees of Caster', 'MZ 8 degrees of Caster', 'Lateral Force')
% ylabel('Normalized MZ and Normalized FY')
% xlabel('Slip Angle')
% title('Normalized Lateral Force and Aligning Moment vs Slip Angle')
% xlim([-10 10])
% ylim([-1.25 1.25])
disp(SAarray(steerEffortlocs3))
disp(SAarray(steerEffortlocs42))
disp(SAarray(steerEffortlocs5))
disp(SAarray(steerEffortlocs6))
disp(SAarray(steerEffortlocs7))
disp(SAarray(steerEffortlocs8))

figure
plot(SAarray, FY)
title('Lateral Force vs Slip Angle')
ylabel(' FY (lbs)')
xlabel('SA (degrees)')

end

