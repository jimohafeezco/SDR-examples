close all; clc;  clear; %clear classes;

%Create user interfase object for SRD
SRD = SRDuserinterface();

timerVal = tic;
SRD.DeriveEquationsForSimulation('UseCasadi', false, 'ToLinearize', true, 'ToSimplify', true, ...
    'ToRecreateSymbolicEngine', true, 'dissipation_coefficients', [], ...
    'NumberOfWorkers', 8, 'ToUseParallelizedSimplification', false, 'ToOptimizeFunctions', true);
toc(timerVal);