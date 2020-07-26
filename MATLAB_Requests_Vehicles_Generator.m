% script to run all the assignment problem.

% This scrit runs in the correct order all the needed scripts to obtain the
% optimal assignment.
% tic-toc does not account for the generation of the plots in evaluationg
% the execution time.

% IF YOU WANT TO SEE THE PLOTS OF ALL THE STEPS (RV_graph, RTV-graph, map
% with requests and vehicles positions) JUST COMMENT THE 'return'

% PERFORMANCE: with 100 tasks and 30 vehicles I got about 1.6 seconds to
% obtain the optimal assignment in my laptop. Implemented in C++ (or outside matlab in general)
% maybe it will take even less time

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This part is of interest only when I need to generate new tasks or generate vehicles

N = 20;
num_v = 15;
max_cap = 5;
old_N = 0;
% trips initialization as structures
% - we should then add the number of passengers
for i=1:floor(N/4)
    R(i).xo = 120*rand(1,1);
    R(i).yo = 120*rand(1,1);
    R(i).xd = R(i).xo + 50*rand(1,1);
    R(i).yd = R(i).yo + 50*rand(1,1);
    R(i).st = 1;
    R(i).at = R(i).st+tt([R(i).xo,R(i).yo],[R(i).xd,R(i).yd]);
    R(i).pass = randi(2,1); % number of passengers: a random number form 1 to 2
    R(i).ID = old_N + i; %%%% request ID
    R(i).status = 0;
    R(i).priority = 0;
end
for i=floor(N/4)+1:floor(N/2)
    R(i).xo = 120*rand(1,1);
    R(i).yo = 120*rand(1,1);
    R(i).xd = R(i).xo - 50*rand(1,1);
    R(i).yd = R(i).yo - 50*rand(1,1);
    R(i).st = 1;
    R(i).at = R(i).st+tt([R(i).xo,R(i).yo],[R(i).xd,R(i).yd]);
    R(i).pass = randi(2,1);
    R(i).ID = old_N + i;
    R(i).status = 0;
    R(i).priority = 0;
end
for i=floor(N/2)+1:floor(3*N/4)
    R(i).xo = 120*rand(1,1);
    R(i).yo = 120*rand(1,1);
    R(i).xd = R(i).xo + 50*rand(1,1);
    R(i).yd = R(i).yo - 50*rand(1,1);
    R(i).st = 1;
    R(i).at = R(i).st+tt([R(i).xo,R(i).yo],[R(i).xd,R(i).yd]);
    R(i).pass = randi(2,1);
    R(i).ID = old_N + i;
    R(i).status = 0;
    R(i).priority = 0;
end
for i=floor(3*N/4)+1:N
    R(i).xo = 120*rand(1,1);
    R(i).yo = 120*rand(1,1);
    R(i).xd = R(i).xo - 50*rand(1,1);
    R(i).yd = R(i).yo + 50*rand(1,1);
    R(i).st = 1;
    R(i).at = R(i).st+tt([R(i).xo,R(i).yo],[R(i).xd,R(i).yd]);
    R(i).pass = randi(2,1);
    R(i).ID = old_N + i;
    R(i).status = 0;
    R(i).priority = 0;
end

fileID1 = fopen('Requests_Data.txt','a');
for i=1:N
    %fprintf(fileID1, '%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\r\n', [R(i).xo, R(i).yo,...
        %R(i).xd, R(i).yd, R(i).st, R(i).at, R(i).pass, (R(i).ID-1), R(i).status R(i).priority]);
    fprintf(fileID1, '%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\r\n', [R(i).xo, R(i).yo,...
        R(i).xd, R(i).yd, R(i).st, R(i).at, R(i).pass, (R(i).ID-1), R(i).status R(i).priority]);
end
fclose(fileID1);

%load('R_k_k_kkk.mat');
new_R = R; % these are the actual new tasks
new_R_exist = 1; % To say that new tasks have arrived
clear R; % I do this only to not change the name to the code ubove
% initialization of vehicles

% (the vehicles should be generated once, and then just make their status 0 when free)

for i=1:floor(num_v/4);
    V(i).x = 70*rand(1,1);
    V(i).y = 20*rand(1,1);
    V(i).c = max_cap;
    V(i).ID = i; %%%% vehicle ID
    V(i).status = 0;
end
for i=floor(num_v/4)+1:floor(num_v/2)
    V(i).x = 80+20*rand(1,1);
    V(i).y = 80+20*rand(1,1);
    V(i).c = max_cap;
    V(i).ID = i;
    V(i).status = 0;
end
for i=floor(num_v/2)+1:floor(3*num_v/4)
    V(i).x = 20+20*rand(1,1);
    V(i).y = 80+20*rand(1,1);
    V(i).c = max_cap;
    V(i).ID = i;
    V(i).status = 0;
end
for i=floor(3*num_v/4)+1:num_v
    V(i).x = 50+20*rand(1,1);
    V(i).y = 50+100*rand(1,1);
    V(i).c = max_cap;
    V(i).ID = i;
    V(i).status = 0;
end

fileID2 = fopen('Vehicles_Data.txt','a');
for i=1:num_v
    %fprintf(fileID2, '%6.3f %6.3f %6.3f %6.3f %6.3f\r\n', [V(i).x, V(i).y,...
        %V(i).c, (V(i).ID-1), V(i).status]);
    fprintf(fileID2, '%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\r\n', [V(i).x, V(i).y,...
        V(i).c, (V(i).ID-1), V(i).status]);
end
fclose(fileID2);