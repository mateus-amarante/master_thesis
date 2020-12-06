% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physics',...
    'config','plot','trajectories','trajectory_planning','optimization_functions');

% Pick your config function
% [physics_p, control_p, traj_p, plot_p] = nested2d_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_config();
% [physics_p, control_p, traj_p, plot_p] = nested2d_slung_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_slung_config();
[physics_p, control_p, traj_p, plot_p] = smc3d_slung_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_slung_config2();
% [physics_p, control_p, traj_p, sim_p, plot_p] = shaped_smc3d_slung_config();
% optimization_function = @nested2d_xzerror_optfun;
% optimization_function = @smc2d_xzerror_optfun;
optimization_function = @final_smc3d_multiobj_optfun;

% warnId = 'MATLAB:ode45:IntegrationTolNotMet';
% warnstate = warning('warning', warnId);

options = optimoptions('gamultiobj','Display','iter','UseParallel',true,'OutputFcn',@(op,s,fl)outputfun(op,s,fl),...%,seed),...
    'PopulationSize',60,'MaxGenerations',100,'MaxStallGenerations',100,...
    'PlotFcn',{@gaplotpareto,@gaplotstopping,@gaplotparetodistance});%'gaplotbestf' | 'gaplotbestindiv'

n_input = 4;
lower = .1;
upper = 10.;
lb = ones(1,n_input)*lower;
ub = ones(1,n_input)*upper;

% lb([10,12]) = -upper;
% ub([10,12]) = -lower;

% lb(4:5) = -100;

[xx,fval,exitflag,output,population,scores] = gamultiobj(@(x,nvar)optimization_function(x,physics_p,control_p,traj_p,sim_p),...
    n_input,[],[],[],[],lb,ub,[],options);

