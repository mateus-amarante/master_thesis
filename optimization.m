% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physical_params',...
    'config','plot','trajectories','trajectory_planning','optimization_functions');

% Pick your config function
% [physics_p, control_p, traj_p, plot_p] = nested2d_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_config();
% [physics_p, control_p, traj_p, plot_p] = nested2d_slung_config();
[physics_p, control_p, traj_p, plot_p] = smc2d_slung_config();
% optimization_function = @nested2d_xzerror_optfun;
optimization_function = @smc2d_xzerror_optfun;
 
% warnId = 'MATLAB:ode45:IntegrationTolNotMet';
% warnstate = warning('warning', warnId);

options = optimoptions('ga','Display','iter','UseParallel',true,'OutputFcn',@(op,s,fl)outputfun(op,s,fl),...%,seed),...
    'PopulationSize',50,'MaxGenerations',50,'MaxStallGenerations',100,...
    'PlotFcn',{@gaplotbestf,@gaplotstopping,@gaplotbestindiv});%'gaplotbestf' | 'gaplotbestindiv'

lb = ones(1,control_p.n_input)*.5;
ub = ones(1,control_p.n_input)*50;

[xx,fval,exitflag,output,population,scores] = ga(@(x,nvar)optimization_function(x,physics_p,control_p,traj_p),...
    control_p.n_input,[],[],[],[],lb,ub,[],options);

