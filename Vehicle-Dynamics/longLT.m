function [deltaW, a] = longLT(W, a, H, l, d, n, A)
%% Longitudinal Load Transfer Calculator
% Brennan Burris
% brennanburris@gmail.com
% 9/16/2022
 %% you are gonna need to download TM the tire model lol
%% definitions
% W = total vehicle weight 
% acceleration in G
% center of gravity height
% l = wheelbase
% d = weight over the front axle ie .4 for 40/60 split
% n = iteration number
% A = axle front or rear. 1 = front, 2 = rear

i = 1;
if A == 1
    for i = 1:1:n
        deltaW = W*a*H/l;
        [NFX, NFY] = TM( (deltaW+(W*d))/2, 0, .18, 0);
        a = NFX;
    end
elseif A == 2
    for i = i:1:n
        deltaW= W*a*H/l;
        [NFX, NFY] = TM((deltaW+(W*(1-d)))/2, 0, .18, 0);
        a = NFX;
    end
else
    error('please enter either 1 or 2 for A')
end 
    
    
end


