% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physics',...
    'config','plot','trajectories','trajectory_planning', 'analysis');

set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');

set(0, 'DefaultLineLineWidth', 1.25);
set(0,'DefaultAxesFontSize',14);
% set(0,'DefaultAxesTitleFontWeight','normal');
