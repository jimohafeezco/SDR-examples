close all; clc; clear all;

path{4} = 'anymal';

path{2} = 'aliengo'; %% 4 legs
path{3} = 'cheetah';
path{5} = 'anymal';
path{1} = 'laikago';
% path{5} = 'hexapod';    %%6 legs
% path{1}='/home/hafeez/MATLAB_ws/SDR-examples/Three link'

controllers = {'CTC', 'CLQR'};
% controllers = {'Nested_QP'};

metric = zeros(length(path),length(controllers));
omega = 25;

parameters.Kp =500;
parameters.Kd = 100;


for i = 1:length(path)
    for j =1: length(controllers)
        current_dir = pwd;
        cd(path{i});
        [cost]=SetupStep5_Simulation(parameters, controllers(j));
        metric(i,j) = cost
        cd(current_dir);
    end
end
