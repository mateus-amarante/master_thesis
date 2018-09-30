% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physical_params',...
    'config','plot','trajectories','trajectory_planning');

set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 