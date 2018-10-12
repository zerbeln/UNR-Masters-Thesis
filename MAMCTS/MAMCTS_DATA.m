%Nicholas Zerbel
%MAMCTS Data

close all; clear all; clc;
format compact

%% Experimental Parameters
parameters = load('test_parameters.txt');
xdim = parameters(1)
ydim = parameters(2)
epsilon = parameters(3)
mc_iterations = parameters(4)
stat_runs = parameters(5)
max_runs = parameters(6)
starting_agents = parameters(7)
max_agents = parameters(8)
increment = parameters(9)
obs_dist = parameters(10)
rollout_its = parameters(11)
goal_reward = parameters(12)
penalty = parameters(13)
rollout_reward = parameters(14)
n = stat_runs; %Number of Columns
m = ((max_agents-starting_agents)/increment) + 1; %Number of Rows

local_data(1:m, 1:n) = 0;
global_data(1:m, 1:n) = 0;
difference_data(1:m, 1:n) = 0;

local_data = load('local_its.txt');
global_data = load('global_its.txt');
difference_data = load('difference_its.txt');

l_avg(1:m) = 0;
g_avg(1:m) = 0;
d_avg(1:m) = 0;

for i = 1:m
    l_avg(i) = mean(local_data(i,:));
    g_avg(i) = mean(global_data(i,:));
    d_avg(i) = mean(difference_data(i,:));
end

%Successful Runs
l_succ(1:m) = 30;
g_succ(1:m) = 30;
d_succ(1:m) = 30;

for i = 1:m
    for j = 1:n
        if local_data(i,j) >= max_runs
            l_succ(i) = l_succ(i)-1;
        end
        if global_data(i,j) >= max_runs
            g_succ(i) = g_succ(i)-1;
        end
        if difference_data(i,j) >= max_runs
            d_succ(i) = d_succ(i)-1;
        end
    end
end

%% Statistics
disp('Number of successful runs out of 30 stat runs:')
s_runs_local = l_succ
s_runs_global = g_succ
s_runs_difference = d_succ

disp('Average number of successful runs overall:')
avg_local = mean(l_succ)
avg_global = mean(g_succ)
avg_dif = mean(d_succ)

disp('Number of learning episodes needed for completion:')
n_eps_local = l_avg
n_eps_global = g_avg
n_eps_difference = d_avg

disp('Average number of learning episodes needed overall')
e_avg_local = mean(l_avg)
e_avg_global = mean(g_avg)
e_avg_difference = mean(d_avg)

disp('Percentage Reliability')
l_rel = 100*(sum(l_succ)/(m*30))
g_rel = 100*(sum(g_succ)/(m*30))
d_rel = 100*(sum(d_succ)/(m*30))



%% Plots
n_agents = [starting_agents:increment:max_agents];
plot(n_agents, l_succ)
xlabel('Number of Agents')
ylabel('Number of Succesful Stat Runs')
title('Number of Successful Runs')
hold on
plot(n_agents, g_succ)
plot(n_agents, d_succ)
legend('Local', 'Global', 'Difference')

figure()
plot(n_agents, l_avg)
xlabel('Number of Agents')
ylabel('Number of Episodes')
title('Learning Episodes For Completion')
hold on
plot(n_agents, g_avg)
plot(n_agents, d_avg)
legend('Local', 'Global', 'Difference')
